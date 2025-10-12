# Simple Brightness Control System

Two working modes: **NORMAL** and **BLUE** modes. The **USER button**
switches the working mode.

A **brightness measure** must be taken every **2 seconds** in **NORMAL
mode**. Depending on the read value, the **RGB LED**:
    - turns **RED** if brightness is below 33%.
    - turns **YELLOW** if brightness is between 33 and 66%.
    - turns **GREEN** if brightness is above 66%.

The **RGB LED** may be controlled either using digital or bus outputs.

In **BLUE mode** the **RGB LED** is **BLUE** and **no brightness measures** are taken until back in **NORMAL mode**.

In both modes the information is displayed on the Zephyr terminal.

To control **brightness measure** and **serial connexion** with an independent thread (using a new file for it).

If a long press (1 second long) is detected in the **USER button**, the system enters **OFF mode**. In such case, **RGB LED** turns OFF and **serial connexion** sends the message “System OFF”.