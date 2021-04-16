## Milestone 1
---

Your goal in this project is to first use the HiFive board to send I2c commands to PCA9695 to drive the servo motor (for steering).

---

* HiFive communnicates with I2C to the PCA9685. 
* The ESC is controlled by the PWM signal provied by the PCA9685.
`DC = cycles_off - Cycles_on`