/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/a64_memorymap.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/// Enable MIPI Display Physical Layer (DPHY).
/// Based on https://lupyuen.github.io/articles/dsi#appendix-enable-mipi-display-physical-layer-dphy
int a64_mipi_dphy_enable(void);
