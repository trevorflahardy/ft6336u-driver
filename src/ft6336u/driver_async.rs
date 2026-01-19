//! Async driver implementation for the FT6336U touch controller.
//!
//! This module contains the async driver struct and all its methods
//! for interacting with the FT6336U hardware using async I2C operations.
//!
//! This module is only available when the `async` feature is enabled.

use embedded_hal_async::i2c::I2c;

use super::constants::*;
use super::error::Error;
use super::types::*;

/// FT6336U capacitive touch controller driver with async I2C interface
///
/// This driver provides a high-level async interface to the FT6336U touch controller,
/// supporting touch scanning, gesture detection, and configuration of various
/// operating parameters.
///
/// # Examples
///
/// Basic usage with polling:
///
/// ```rust,no_run
/// # use embedded_hal_async::i2c::I2c;
/// # use core::convert::Infallible;
/// # struct MockI2c;
/// # impl embedded_hal::i2c::ErrorType for MockI2c {
/// #     type Error = Infallible;
/// # }
/// # impl I2c for MockI2c {
/// #     async fn write(&mut self, _: u8, _: &[u8]) -> Result<(), Self::Error> { Ok(()) }
/// #     async fn read(&mut self, _: u8, _: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
/// #     async fn write_read(&mut self, _: u8, _: &[u8], _: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
/// #     async fn transaction(&mut self, _: u8, _: &mut [embedded_hal_async::i2c::Operation<'_>]) -> Result<(), Self::Error> { Ok(()) }
/// # }
/// # async fn example() {
/// # let i2c = MockI2c;
/// use ft6336u_driver::FT6336U;
///
/// let mut touch = FT6336U::new(i2c);
///
/// // Poll for touch events in a loop
/// loop {
///     let data = touch.scan().await.unwrap();
///
///     for i in 0..data.touch_count as usize {
///         let point = &data.points[i];
///         // Process touch point...
///     }
/// }
/// # }
/// ```
///
/// Reading device information:
///
/// ```rust,no_run
/// # use embedded_hal_async::i2c::I2c;
/// # use core::convert::Infallible;
/// # struct MockI2c;
/// # impl embedded_hal::i2c::ErrorType for MockI2c {
/// #     type Error = Infallible;
/// # }
/// # impl I2c for MockI2c {
/// #     async fn write(&mut self, _: u8, _: &[u8]) -> Result<(), Self::Error> { Ok(()) }
/// #     async fn read(&mut self, _: u8, _: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
/// #     async fn write_read(&mut self, _: u8, _: &[u8], _: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
/// #     async fn transaction(&mut self, _: u8, _: &mut [embedded_hal_async::i2c::Operation<'_>]) -> Result<(), Self::Error> { Ok(()) }
/// # }
/// # async fn example() {
/// # let i2c = MockI2c;
/// use ft6336u_driver::FT6336U;
///
/// let mut touch = FT6336U::new(i2c);
///
/// // Read device information (these will return errors with MockI2c)
/// // let chip_id = touch.read_chip_id().await.unwrap();
/// // let firmware_id = touch.read_firmware_id().await.unwrap();
/// # }
/// ```
pub struct FT6336U<I2C> {
    /// I2C bus for communicating with the touch controller
    i2c: I2C,
    /// Cached touch point data from last scan
    touch_data: TouchData,
}

impl<I2C> FT6336U<I2C>
where
    I2C: I2c,
{
    /// Create a new FT6336U driver instance
    ///
    /// # Arguments
    /// * `i2c` - I2C bus instance that implements embedded_hal_async::i2c::I2c
    ///
    /// # Note
    /// The reset and interrupt pins should be managed by the AW9523B GPIO expander
    /// or by the calling code before creating this driver instance.
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            touch_data: TouchData::default(),
        }
    }

    // =========================================================================
    // Private I2C Helper Methods
    // =========================================================================

    /// Read a single byte from a register
    async fn read_byte(&mut self, addr: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8; 1];
        self.i2c.write_read(I2C_ADDR, &[addr], &mut buf).await?;
        Ok(buf[0])
    }

    /// Write a single byte to a register
    async fn write_byte(&mut self, addr: u8, data: u8) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(I2C_ADDR, &[addr, data]).await?;
        Ok(())
    }

    // =========================================================================
    // Device Mode Register Methods
    // =========================================================================

    /// Read the current device operating mode
    ///
    /// # Returns
    /// The device mode (Working or Factory)
    pub async fn read_device_mode(&mut self) -> Result<u8, Error<I2C::Error>> {
        let val = self.read_byte(ADDR_DEVICE_MODE).await?;
        Ok((val & 0x70) >> 4)
    }

    /// Write the device operating mode
    ///
    /// # Arguments
    /// * `mode` - The desired device mode
    pub async fn write_device_mode(&mut self, mode: DeviceMode) -> Result<(), Error<I2C::Error>> {
        self.write_byte(ADDR_DEVICE_MODE, mode.to_register()).await
    }

    // =========================================================================
    // Gesture and Touch Status Methods
    // =========================================================================

    /// Read the gesture ID register
    ///
    /// # Returns
    /// Gesture ID value
    pub async fn read_gesture_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_GESTURE_ID).await
    }

    /// Read the touch detection status register
    ///
    /// # Returns
    /// Raw TD_STATUS register value
    pub async fn read_td_status(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_TD_STATUS).await
    }

    /// Read the number of detected touch points
    ///
    /// # Returns
    /// Number of touch points (0-2)
    pub async fn read_touch_number(&mut self) -> Result<u8, Error<I2C::Error>> {
        let val = self.read_byte(ADDR_TD_STATUS).await?;
        Ok(val & 0x0F)
    }

    // =========================================================================
    // Touch Point 1 Methods
    // =========================================================================

    /// Read X coordinate of touch point 1
    ///
    /// # Returns
    /// X coordinate (0-4095, 12-bit value)
    pub async fn read_touch1_x(&mut self) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(I2C_ADDR, &[ADDR_TOUCH1_X], &mut buf)
            .await?;
        Ok((((buf[0] & 0x0F) as u16) << 8) | (buf[1] as u16))
    }

    /// Read Y coordinate of touch point 1
    ///
    /// # Returns
    /// Y coordinate (0-4095, 12-bit value)
    pub async fn read_touch1_y(&mut self) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(I2C_ADDR, &[ADDR_TOUCH1_Y], &mut buf)
            .await?;
        Ok((((buf[0] & 0x0F) as u16) << 8) | (buf[1] as u16))
    }

    /// Read event type of touch point 1
    ///
    /// # Returns
    /// Event type (0=down, 1=up, 2=contact)
    pub async fn read_touch1_event(&mut self) -> Result<u8, Error<I2C::Error>> {
        let val = self.read_byte(ADDR_TOUCH1_EVENT).await?;
        Ok(val >> 6)
    }

    /// Read ID of touch point 1
    ///
    /// # Returns
    /// Touch point ID (0 or 1)
    pub async fn read_touch1_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        let val = self.read_byte(ADDR_TOUCH1_ID).await?;
        Ok(val >> 4)
    }

    /// Read weight/pressure of touch point 1
    ///
    /// # Returns
    /// Touch weight value
    pub async fn read_touch1_weight(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_TOUCH1_WEIGHT).await
    }

    /// Read miscellaneous data for touch point 1
    ///
    /// # Returns
    /// Misc data value
    pub async fn read_touch1_misc(&mut self) -> Result<u8, Error<I2C::Error>> {
        let val = self.read_byte(ADDR_TOUCH1_MISC).await?;
        Ok(val >> 4)
    }

    // =========================================================================
    // Touch Point 2 Methods
    // =========================================================================

    /// Read X coordinate of touch point 2
    ///
    /// # Returns
    /// X coordinate (0-4095, 12-bit value)
    pub async fn read_touch2_x(&mut self) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(I2C_ADDR, &[ADDR_TOUCH2_X], &mut buf)
            .await?;
        Ok((((buf[0] & 0x0F) as u16) << 8) | (buf[1] as u16))
    }

    /// Read Y coordinate of touch point 2
    ///
    /// # Returns
    /// Y coordinate (0-4095, 12-bit value)
    pub async fn read_touch2_y(&mut self) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(I2C_ADDR, &[ADDR_TOUCH2_Y], &mut buf)
            .await?;
        Ok((((buf[0] & 0x0F) as u16) << 8) | (buf[1] as u16))
    }

    /// Read event type of touch point 2
    ///
    /// # Returns
    /// Event type (0=down, 1=up, 2=contact)
    pub async fn read_touch2_event(&mut self) -> Result<u8, Error<I2C::Error>> {
        let val = self.read_byte(ADDR_TOUCH2_EVENT).await?;
        Ok(val >> 6)
    }

    /// Read ID of touch point 2
    ///
    /// # Returns
    /// Touch point ID (0 or 1)
    pub async fn read_touch2_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        let val = self.read_byte(ADDR_TOUCH2_ID).await?;
        Ok(val >> 4)
    }

    /// Read weight/pressure of touch point 2
    ///
    /// # Returns
    /// Touch weight value
    pub async fn read_touch2_weight(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_TOUCH2_WEIGHT).await
    }

    /// Read miscellaneous data for touch point 2
    ///
    /// # Returns
    /// Misc data value
    pub async fn read_touch2_misc(&mut self) -> Result<u8, Error<I2C::Error>> {
        let val = self.read_byte(ADDR_TOUCH2_MISC).await?;
        Ok(val >> 4)
    }

    // =========================================================================
    // Mode Parameter Register Methods
    // =========================================================================

    /// Read the touch detection threshold
    ///
    /// # Returns
    /// Threshold value (lower = more sensitive)
    pub async fn read_touch_threshold(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_THRESHOLD).await
    }

    /// Read the filter coefficient
    ///
    /// # Returns
    /// Filter coefficient value
    pub async fn read_filter_coefficient(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_FILTER_COE).await
    }

    /// Read the control mode register
    ///
    /// # Returns
    /// Control mode value
    pub async fn read_ctrl_mode(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_CTRL).await
    }

    /// Write the control mode
    ///
    /// # Arguments
    /// * `mode` - Control mode (KeepActive or SwitchToMonitor)
    pub async fn write_ctrl_mode(&mut self, mode: CtrlMode) -> Result<(), Error<I2C::Error>> {
        self.write_byte(ADDR_CTRL, mode as u8).await
    }

    /// Read the time period to enter monitor mode
    ///
    /// # Returns
    /// Time period value in seconds
    pub async fn read_time_period_enter_monitor(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_TIME_ENTER_MONITOR).await
    }

    /// Read the active mode report rate
    ///
    /// # Returns
    /// Report rate in Hz
    pub async fn read_active_rate(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_ACTIVE_MODE_RATE).await
    }

    /// Read the monitor mode report rate
    ///
    /// # Returns
    /// Report rate in Hz
    pub async fn read_monitor_rate(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_MONITOR_MODE_RATE).await
    }

    // =========================================================================
    // Gesture Parameter Register Methods
    // =========================================================================

    /// Read the radian value for gesture detection
    ///
    /// # Returns
    /// Radian value
    pub async fn read_radian_value(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_RADIAN_VALUE).await
    }

    /// Write the radian value for gesture detection
    ///
    /// # Arguments
    /// * `val` - Radian value to set
    pub async fn write_radian_value(&mut self, val: u8) -> Result<(), Error<I2C::Error>> {
        self.write_byte(ADDR_RADIAN_VALUE, val).await
    }

    /// Read the offset for left/right gesture detection
    ///
    /// # Returns
    /// Offset value
    pub async fn read_offset_left_right(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_OFFSET_LEFT_RIGHT).await
    }

    /// Write the offset for left/right gesture detection
    ///
    /// # Arguments
    /// * `val` - Offset value to set
    pub async fn write_offset_left_right(&mut self, val: u8) -> Result<(), Error<I2C::Error>> {
        self.write_byte(ADDR_OFFSET_LEFT_RIGHT, val).await
    }

    /// Read the offset for up/down gesture detection
    ///
    /// # Returns
    /// Offset value
    pub async fn read_offset_up_down(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_OFFSET_UP_DOWN).await
    }

    /// Write the offset for up/down gesture detection
    ///
    /// # Arguments
    /// * `val` - Offset value to set
    pub async fn write_offset_up_down(&mut self, val: u8) -> Result<(), Error<I2C::Error>> {
        self.write_byte(ADDR_OFFSET_UP_DOWN, val).await
    }

    /// Read the distance for left/right gesture detection
    ///
    /// # Returns
    /// Distance value
    pub async fn read_distance_left_right(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_DISTANCE_LEFT_RIGHT).await
    }

    /// Write the distance for left/right gesture detection
    ///
    /// # Arguments
    /// * `val` - Distance value to set
    pub async fn write_distance_left_right(&mut self, val: u8) -> Result<(), Error<I2C::Error>> {
        self.write_byte(ADDR_DISTANCE_LEFT_RIGHT, val).await
    }

    /// Read the distance for up/down gesture detection
    ///
    /// # Returns
    /// Distance value
    pub async fn read_distance_up_down(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_DISTANCE_UP_DOWN).await
    }

    /// Write the distance for up/down gesture detection
    ///
    /// # Arguments
    /// * `val` - Distance value to set
    pub async fn write_distance_up_down(&mut self, val: u8) -> Result<(), Error<I2C::Error>> {
        self.write_byte(ADDR_DISTANCE_UP_DOWN, val).await
    }

    /// Read the distance for zoom gesture detection
    ///
    /// # Returns
    /// Distance value
    pub async fn read_distance_zoom(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_DISTANCE_ZOOM).await
    }

    /// Write the distance for zoom gesture detection
    ///
    /// # Arguments
    /// * `val` - Distance value to set
    pub async fn write_distance_zoom(&mut self, val: u8) -> Result<(), Error<I2C::Error>> {
        self.write_byte(ADDR_DISTANCE_ZOOM, val).await
    }

    // =========================================================================
    // System Information Methods
    // =========================================================================

    /// Read the library version from the device
    ///
    /// # Returns
    /// 16-bit library version number
    pub async fn read_library_version(&mut self) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0u8; 2];
        self.i2c
            .write_read(I2C_ADDR, &[ADDR_LIBRARY_VERSION_H], &mut buf)
            .await?;
        Ok((((buf[0] & 0x0F) as u16) << 8) | (buf[1] as u16))
    }

    /// Read the chip ID
    ///
    /// # Returns
    /// Chip ID (should be 0x64 for FT6336U)
    pub async fn read_chip_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_CHIP_ID).await
    }

    /// Read the gesture/interrupt mode
    ///
    /// # Returns
    /// G_MODE register value
    pub async fn read_g_mode(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_G_MODE).await
    }

    /// Write the gesture/interrupt mode
    ///
    /// # Arguments
    /// * `mode` - Gesture mode (Polling or Trigger)
    pub async fn write_g_mode(&mut self, mode: GestureMode) -> Result<(), Error<I2C::Error>> {
        self.write_byte(ADDR_G_MODE, mode as u8).await
    }

    /// Read the power mode
    ///
    /// # Returns
    /// Power mode value
    pub async fn read_pwrmode(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_POWER_MODE).await
    }

    /// Read the firmware ID
    ///
    /// # Returns
    /// Firmware ID value
    pub async fn read_firmware_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_FIRMWARE_ID).await
    }

    /// Read the Focaltech ID
    ///
    /// # Returns
    /// Focaltech ID value
    pub async fn read_focaltech_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_FOCALTECH_ID).await
    }

    /// Read the release code ID
    ///
    /// # Returns
    /// Release code ID value
    pub async fn read_release_code_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_RELEASE_CODE_ID).await
    }

    /// Read the device state
    ///
    /// # Returns
    /// Device state value
    pub async fn read_state(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_byte(ADDR_STATE).await
    }

    // =========================================================================
    // High-Level Scan Method
    // =========================================================================

    /// Scan for touch events and update internal touch data
    ///
    /// This is the main method to call periodically or in response to interrupts
    /// to read the current touch state. It reads all touch point data and updates
    /// the internal touch data structure.
    ///
    /// # Returns
    /// TouchData containing the number of touch points and their coordinates/status
    pub async fn scan(&mut self) -> Result<TouchData, Error<I2C::Error>> {
        // Read the number of touch points
        let touch_count = self.read_touch_number().await?;
        self.touch_data.touch_count = touch_count;

        if touch_count == 0 {
            // No touches - mark both points as released
            self.touch_data.points[0].status = TouchStatus::Release;
            self.touch_data.points[1].status = TouchStatus::Release;
        } else if touch_count == 1 {
            // Single touch point
            let id1 = self.read_touch1_id().await? as usize;
            if id1 < 2 {
                // Update status: if previously released, mark as new touch, otherwise streaming
                let prev_status = self.touch_data.points[id1].status;
                self.touch_data.points[id1].status = match prev_status {
                    TouchStatus::Release => TouchStatus::Touch,
                    _ => TouchStatus::Stream,
                };

                // Read coordinates
                self.touch_data.points[id1].x = self.read_touch1_x().await?;
                self.touch_data.points[id1].y = self.read_touch1_y().await?;

                // Mark the other point as released
                let other_id = (!id1) & 0x01;
                self.touch_data.points[other_id].status = TouchStatus::Release;
            }
        } else {
            // Two touch points
            let id1 = self.read_touch1_id().await? as usize;
            if id1 < 2 {
                let prev_status1 = self.touch_data.points[id1].status;
                self.touch_data.points[id1].status = match prev_status1 {
                    TouchStatus::Release => TouchStatus::Touch,
                    _ => TouchStatus::Stream,
                };
                self.touch_data.points[id1].x = self.read_touch1_x().await?;
                self.touch_data.points[id1].y = self.read_touch1_y().await?;
            }

            let id2 = self.read_touch2_id().await? as usize;
            if id2 < 2 {
                let prev_status2 = self.touch_data.points[id2].status;
                self.touch_data.points[id2].status = match prev_status2 {
                    TouchStatus::Release => TouchStatus::Touch,
                    _ => TouchStatus::Stream,
                };
                self.touch_data.points[id2].x = self.read_touch2_x().await?;
                self.touch_data.points[id2].y = self.read_touch2_y().await?;
            }
        }

        Ok(self.touch_data)
    }
}
