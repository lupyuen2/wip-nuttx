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
