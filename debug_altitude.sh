#!/bin/bash
# Debug script to check MSP_ALTITUDE

echo "==================================="
echo "MSP_ALTITUDE Debug Script"
echo "==================================="
echo ""

echo "1. Checking /fc/altitude topic..."
timeout 5 ros2 topic echo /fc/altitude --once
echo ""

echo "2. Checking if fc_comms is running..."
ros2 node list | grep fc_comms
echo ""

echo "3. Checking /fc/msp_status (sensor mask)..."
echo "   Looking for barometer bit (bit 1 = barometer)..."
timeout 5 ros2 topic echo /fc/msp_status --once
echo ""

echo "4. Checking other FC topics..."
echo "   GPS altitude (for comparison):"
timeout 5 ros2 topic echo /fc/gps_fix --field altitude --once
echo ""

echo "5. Raw MSP_STATUS sensor mask:"
timeout 5 ros2 topic echo /fc/msp_status --field data[2] --once
echo ""

echo "==================================="
echo "Barometer check:"
echo "  If data[2] from msp_status = 2 or higher, barometer is detected"
echo "  Bit 0 (1) = Accelerometer"
echo "  Bit 1 (2) = Barometer"
echo "  Bit 2 (4) = Magnetometer"
echo "  Bit 3 (8) = GPS"
echo "  Example: 15 = 1+2+4+8 = all sensors active"
echo "==================================="
