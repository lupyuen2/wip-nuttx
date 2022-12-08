/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/// MIPI DSI Processor-to-Peripheral transaction types:
/// DCS Long Write. See https://lupyuen.github.io/articles/dsi#display-command-set-for-mipi-dsi
#define MIPI_DSI_DCS_LONG_WRITE 0x39

/// DCS Short Write (Without Parameter)
#define MIPI_DSI_DCS_SHORT_WRITE 0x05

/// DCS Short Write (With Parameter)
#define MIPI_DSI_DCS_SHORT_WRITE_PARAM 0x15

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

// Compose MIPI DSI Long Packet. See https://lupyuen.github.io/articles/dsi#long-packet-for-mipi-dsi
// Return the number of bytes in the composed packet.
// Returns -1 if pktlen too short
ssize_t mipi_dsi_long_packet(
  FAR uint8_t *pktbuf,    // Buffer for the Returned Long Packet
  size_t pktlen,    // Buffer Size for the Returned Long Packet
  uint8_t channel,  // Virtual Channel ID
  uint8_t cmd,      // DCS Command
  FAR const uint8_t *txbuf,  // Transmit Buffer
  size_t txlen          // Buffer Length
);

// Compose MIPI DSI Short Packet. See https://lupyuen.github.io/articles/dsi#appendix-short-packet-for-mipi-dsi
// Return the number of bytes in the composed packet.
// Returns -1 if pktlen too short
ssize_t mipi_dsi_short_packet(
  FAR uint8_t *pktbuf,    // Buffer for the Returned Short Packet
  size_t pktlen,    // Buffer Size for the Returned Short Packet
  uint8_t channel,  // Virtual Channel ID
  uint8_t cmd,      // DCS Command
  FAR const uint8_t *txbuf,  // Transmit Buffer
  size_t txlen          // Buffer Length
);
