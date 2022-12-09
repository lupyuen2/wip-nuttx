/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/a64_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/// MIPI DSI Virtual Channel
#define A64_MIPI_DSI_VIRTUAL_CHANNEL 0

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/// Write to MIPI DSI. See https://lupyuen.github.io/articles/dsi#transmit-packet-over-mipi-dsi
/// On Success: Return number of written bytes. On Error: Return negative error code
ssize_t a64_mipi_dsi_write(
    uint8_t channel,  // Virtual Channel ID
    uint8_t cmd,      // DCS Command
    FAR const uint8_t *txbuf,  // Transmit Buffer
    size_t txlen          // Buffer Length
);

/// Enable MIPI DSI Block.
/// Based on https://lupyuen.github.io/articles/dsi#appendix-enable-mipi-dsi-block
int a64_mipi_dsi_enable(void);

/// Start MIPI DSI HSC and HSD. (High Speed Clock Mode and High Speed Data Transmission)
/// Based on https://lupyuen.github.io/articles/dsi#appendix-start-mipi-dsi-hsc-and-hsd
int a64_mipi_dsi_start(void);
