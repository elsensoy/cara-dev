3D CAD model of a 20-DoF humanoid teddy bear robot skeleton. Design follows research-grade joint standards: 3-axis waist (pitch/yaw/roll), two 3-DoF shoulders, two 3-DoF hips, a 3-axis neck, and 1-DoF expressive ears. No tail. The rounded mid-body must include a precise mounting cradle for a Jetson Orin Nano Super and a LiPo battery. Focus on functional mechanical geometry: recessed pockets for standard servos, internal wire-routing channels, and reinforced bolt-hole patterns. Skeleton should have ergonomic, rounded edges to fit inside a plush fabric shell. Ensure the mesh is manifold, 3D-printable, and optimized for high-torque motion and stability. Proportions: wide torso, sturdy leg-base, and expressive head mount.


Mechanical Strategy for Cara

Since we are using the Jetson Orin Nano Super, we have significant compute power, but it also means the mid-body needs specific thermal and structural features:

   *Weight Distribution:* To keep Cara stable while walking or moving, the Orin Nano and the battery should be placed as low as possible in the "belly" of the bear. This lowers the Center of Gravity (CoG).

    *Thermal Venting:* Even though she’s a teddy bear, the Orin Nano Super generates heat. cad design might need manual "slats" or a small 5V fan mount added to the back of the skeleton to prevent the plush fur from trapping too much heat.

    *Servo Selection:* For the 20-joint setup, I recommend using high-torque metal gear servos for the waist and hips, as they will support the entire weight of the Orin and the upper body.

### A Note on the Ears

By assigning 2 servos to the ears, we can program "moods" (e.g., ears pinned back for "sad/scared," twitching for "listening/curious"). This is a huge part of what will make Cara feel sentient rather than just a machine.


Safety First: The Orin Nano Super Wiring

Using a Jetson Orin Nano Super, the primary goal is protecting the carrier board from "back-EMF" (voltage spikes) from those 20 servos.

The "Isolation" Diagram:
Plaintext

[ 12V LiPo Battery ] 
      |
      +-----> [ High-Current 5V/6V BEC ] ------> [ PCA9685 Power Terminal ]
      |                                                  |
      +-----> [ Jetson Power Regulator ] ------> [ Jetson Orin Nano Super ]
                                                         |
                                                 [ I2C SDA/SCL Pins ]
                                                         |
                                                 [ PCA9685 Logic Pins ]

**CRITICAL: Connect the Ground (GND) of the BEC to the GND of the Jetson.**
