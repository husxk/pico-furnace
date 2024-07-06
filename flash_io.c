#include <hardware/flash.h>
#include <hardware/sync.h>
#include <pico/time.h>

#include <string.h>

#include "common.h"
#include "flash_io.h"

#define FLASH_PAGE_COUNT_IN_SECTOR FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE
#define FLASH_MAX_PAGE_INDEX FLASH_PAGE_COUNT_IN_SECTOR - 1

// Identifier to distinguish between random bytes and our data in flash memory.
#define TAG 0xAAAAAAAAAAAAAAAA

// this symbol is defined in the linker script (memmap.ld)
extern size_t
FLASH_DEVICE_DATA_START;

enum __attribute__((packed))
init_targets {
  MAGNETRON = 1 << 0,
  WATER     = 1 << 1,
  PILOT     = 1 << 2,
  MAPPER    = 1 << 3,
  SHUTTER   = 1 << 4,
  STIRER    = 1 << 5,

  NONE      = 1 << 7      /*
                            * NONE must be last.
                            * This is done to distinguish "-1" from valid init_targets.
                            * By default, flash memory is set to "-1".
                            */
};

typedef struct __attribute__((packed))
{
  const int64_t           tag;
  const enum init_targets targets;
  uint8_t                 log_bits;
  uint8_t                 pwm_level;
  int                     ceiling_pwm;

#if CONFIG_WATER
  uint8_t pwm_water;
#endif

#if CONFIG_MAGNETRON
  uint8_t         magnetron_pulse_count;
  absolute_time_t magnetron_deadline;
#endif

#if CONFIG_AUTO == CONFIG_AUTO_PILOT || CONFIG_AUTO == CONFIG_AUTO_MAPPER
  bool pilot_is_enabled;
  int  pilot_des_temp;
#endif

#if CONFIG_AUTO == CONFIG_AUTO_MAPPER
  bool mapper_is_enabled;
  int  mapper_max_pwm_temp;
#endif

} flash_valid_data_t;

#define FLASH_VALID_DATA_SIZE sizeof(flash_valid_data_t)

typedef struct __attribute__((packed))
{
  flash_valid_data_t data;
  uint8_t            padding[FLASH_PAGE_SIZE - FLASH_VALID_DATA_SIZE];
} flash_data_t;

// flash_data_t must be the same size as a page. It is required by the algorithm
static_assert(sizeof(flash_data_t) == FLASH_PAGE_SIZE, "FLASH ERROR: sizeof(flash_data_t) != FLASH_PAGE_SIZE\n");

typedef struct
{
  const flash_data_t* ptr;
  int8_t              current_index;
} flash_ptr_t;

static const enum init_targets
target = 0
  #if CONFIG_MAGNETRON
    + MAGNETRON
  #endif

  #if CONFIG_WATER
    + WATER
  #endif

  #if CONFIG_AUTO == CONFIG_AUTO_PILOT || CONFIG_AUTO == CONFIG_AUTO_MAPPER
  + PILOT
  #endif

  #if CONFIG_AUTO == CONFIG_AUTO_MAPPER
  + MAPPER
  #endif

  #if CONFIG_SHUTTER
  + SHUTTER
  #endif

  #if CONFIG_STIRER
  + STIRER
  #endif

;

static flash_ptr_t flash_ptr_1 =
  {
    .ptr           = (flash_data_t*)&FLASH_DEVICE_DATA_START,
    .current_index = -1,
  };

static flash_ptr_t flash_ptr_2 =
  {
    .ptr           = (void*)&FLASH_DEVICE_DATA_START + FLASH_SECTOR_SIZE,
    .current_index = -1,
  };

static flash_valid_data_t flash_last_written =
  {
    .targets = target,
    .tag     = TAG,
  };

static flash_data_t flash_current =
  {
    .data.targets = target,
    .data.tag     = TAG,
  };

static bool
flash_is_page_empty(const int8_t* ptr)
{
  for(int i = 0; i < FLASH_PAGE_SIZE; i++)
  {
    if(*ptr != -1)
      return false;
    ptr++;
  }
  return true;
}

static void
flash_find_last_blocks()
{
  bool empty_1 = true;
  bool empty_2 = true;

  for(int i = FLASH_MAX_PAGE_INDEX; i >= 0; i--)
  {
    const int8_t* ptr_1 = (int8_t*)&flash_ptr_1.ptr[i];
    const int8_t* ptr_2 = (int8_t*)&flash_ptr_2.ptr[i];

    if(empty_1)
    {
      if(!flash_is_page_empty(ptr_1))
      {
        flash_ptr_1.current_index = i;
        empty_1 = false;
      }
    }

    if(empty_2)
    {
      if(!flash_is_page_empty(ptr_2))
      {
        flash_ptr_2.current_index = i;
        empty_2 = false;
      }
    }
  }
}

static void
flash_read(flash_ptr_t* flash_ptr_, furnace_context_t* ctx)
{
  const flash_valid_data_t* flash_ptr = &flash_ptr_->ptr[flash_ptr_->current_index].data;

  if(flash_ptr->targets != target)
    return;

  ctx->log_bits    = flash_ptr->log_bits;
  ctx->pwm_level   = flash_ptr->pwm_level;
  ctx->ceiling_pwm = flash_ptr->ceiling_pwm;

#if CONFIG_WATER
  ctx->pwm_water = flash_ptr->pwm_water;
#endif

#if CONFIG_MAGNETRON
  ctx->pulse_count        = flash_ptr->magnetron_pulse_count;
  ctx->magnetron_deadline = flash_ptr->magnetron_magnetron_deadline;
#endif

#if CONFIG_AUTO == CONFIG_AUTO_PILOT || CONFIG_AUTO == CONFIG_AUTO_MAPPER
  ctx->pilot.des_temp   = flash_ptr->pilot_des_temp;
  ctx->pilot.is_enabled = flash_ptr->pilot_is_enabled;
#endif

#if CONFIG_AUTO == CONFIG_AUTO_MAPPER
  ctx->mapper.max_pwm_temp = flash_ptr->mapper_max_pwm_temp;
  ctx->mapper.is_enabled   = flash_ptr->mapper_is_enabled;
#endif
}

enum flash_valid
{
  VALID_1    = 1 << 0,
  VALID_2    = 1 << 1,
  BOTH_VALID = 1 << 2,
  INVALID    = 1 << 3,
};

static void
flash_clear_sector(const void* ptr)
{
  const uint32_t ints = save_and_disable_interrupts();
  flash_range_erase((uint32_t)ptr - XIP_BASE, FLASH_SECTOR_SIZE);
  restore_interrupts(ints);
}

static void
flash_fix_blocks()
{
  const int8_t index_1 = flash_ptr_1.current_index;
  const int8_t index_2 = flash_ptr_2.current_index;

  /*
   * Special cases when the algorithm expects that memory is erased and
   * prepared for writting.
   * These invalid states may result from power loss.
   */

  // if index_2 == FLASH_MAX_PAGE_INDEX, index_1 is expected to be -1
  if(index_1 == FLASH_MAX_PAGE_INDEX &&
     index_2 == FLASH_MAX_PAGE_INDEX)
  {
    flash_clear_sector(flash_ptr_1.ptr);
    return;
  }

  // if index_1 == 0, index_2 is expected to be -1
  if(index_1 == 0 &&
     index_2 == FLASH_MAX_PAGE_INDEX)
  {
    flash_clear_sector(flash_ptr_2.ptr);
    return;
  }

  /*
   * check if flash memory is in a valid state
   */
  if( index_1 == index_2                               ||
      index_1 == index_2 + 1                           ||
     (index_1 == -1 && index_2 == FLASH_MAX_PAGE_INDEX)
    )
  {
    return;
  }

    flash_clear_sector(flash_ptr_1.ptr);
    flash_clear_sector(flash_ptr_2.ptr);

    flash_ptr_1.current_index = -1;
    flash_ptr_2.current_index = -1;
    return;
}

static enum flash_valid
flash_validate_blocks()
{
  flash_fix_blocks();
  enum flash_valid valid = INVALID;

  const int f1_index = flash_ptr_1.current_index;
  const int f2_index = flash_ptr_2.current_index;

  if(f1_index != -1)
  {
    if(flash_ptr_1.ptr[f1_index].data.tag     == TAG &&
       flash_ptr_1.ptr[f1_index].data.targets == target)
    {
      valid = VALID_1;
    }
  }

  if(f2_index != -1)
  {
    if(flash_ptr_2.ptr[f2_index].data.tag     == TAG &&
       flash_ptr_2.ptr[f2_index].data.targets == target)
    {
      if(valid == VALID_1)
        valid = BOTH_VALID;
      else
        valid = VALID_2;
    }
  }

  return valid;
}

static void
flash_update_lookup(flash_valid_data_t* lookup, furnace_context_t* ctx)
{
  lookup->log_bits    = ctx->log_bits;
  lookup->pwm_level   = ctx->pwm_level;
  lookup->ceiling_pwm = ctx->ceiling_pwm;

#if CONFIG_WATER
  lookup->pwm_water = ctx->pwm_water;
#endif

#if CONFIG_MAGNETRON
  lookup->magnetron_pulse_count = ctx->pulse_count;
  lookup->magnetron_deadline    = ctx->magnetron_deadline;
#endif

#if CONFIG_AUTO == CONFIG_AUTO_PILOT || CONFIG_AUTO == CONFIG_AUTO_MAPPER
  lookup->pilot_is_enabled = ctx->pilot.is_enabled;
  lookup->pilot_des_temp   = ctx->pilot.des_temp;
#endif

#if CONFIG_AUTO == CONFIG_AUTO_MAPPER
  lookup->mapper_is_enabled   = ctx->mapper.is_enabled;
  lookup->mapper_max_pwm_temp = ctx->mapper.max_pwm_temp;
#endif

}

static void
flash_write_(flash_ptr_t* flash_ptr)
{
  const uint32_t ints = save_and_disable_interrupts();
  flash_range_program((uint32_t)&flash_ptr->ptr[flash_ptr->current_index] - XIP_BASE,
                       (uint8_t*)&flash_current,
                       FLASH_PAGE_SIZE);
  restore_interrupts(ints);
}

static void
flash_write()
{
  if(flash_ptr_1.current_index == flash_ptr_2.current_index)
  {
    flash_ptr_1.current_index++;
    flash_ptr_1.current_index %= FLASH_PAGE_COUNT_IN_SECTOR;
    flash_write_(&flash_ptr_1);

    if(flash_ptr_2.current_index == FLASH_MAX_PAGE_INDEX)
      flash_clear_sector(flash_ptr_2.ptr);
  }
  else
  {
    flash_ptr_2.current_index++;
    flash_ptr_2.current_index %= FLASH_PAGE_COUNT_IN_SECTOR;
    flash_write_(&flash_ptr_2);

    if(flash_ptr_1.current_index == FLASH_MAX_PAGE_INDEX)
      flash_clear_sector(flash_ptr_1.ptr);
  }
}

void
init_flash(furnace_context_t* ctx)
{
  flash_find_last_blocks();

  switch(flash_validate_blocks())
  {
    case VALID_1:
    {
      flash_read(&flash_ptr_1, ctx);
    } break;

    case VALID_2:
    {
      flash_read(&flash_ptr_2, ctx);
    } break;

    case BOTH_VALID:
    {
      if(flash_ptr_1.current_index == flash_ptr_2.current_index)
        flash_read(&flash_ptr_2, ctx);
      else
        flash_read(&flash_ptr_1, ctx);
    } break;
  }

  flash_update_lookup(&flash_last_written, ctx);
  flash_update_lookup(&flash_current.data, ctx);

  ctx->flash_deadline = make_timeout_time_ms(FLASH_WRITE_MS);
}

void
do_flash_work(furnace_context_t* ctx)
{
  const absolute_time_t now = get_absolute_time();
  if(now < ctx->flash_deadline)
    return;

  flash_update_lookup(&flash_current.data, ctx);
  if(memcmp(&flash_current.data, &flash_last_written, sizeof(flash_valid_data_t)) != 0)
  {
    flash_write();
    flash_update_lookup(&flash_last_written, ctx);
  }

  ctx->flash_deadline = make_timeout_time_ms(FLASH_WRITE_MS);
}
