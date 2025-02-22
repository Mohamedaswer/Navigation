# Butler Robot for French Door Café

## Project Overview
The French Door Café has requested a robotic butler to automate food delivery to customers, reducing employee costs and increasing efficiency. This robot operates using the **ROS Navigation Stack** in **ROS 1 Noetic** with a custom **Action Server** to handle task execution.

**Workflow:**
1. The robot starts at its **home position**.
2. Upon receiving an order, it moves to the **kitchen** to collect the food.
3. The robot then delivers the food to the assigned **table (table1, table2, table3)**.
4. After completing the task, it **returns home**.
5. The robot handles **multiple orders**, **timeouts**, **cancellations**, and **confirmation requirements** dynamically.

---
## **Package Structure**
This project consists of the following ROS packages:
- **rur_detailed_description** - Contains the robot's URDF model.
- **rur_gazebo** - Contains the **rur_postoffice.launch** file, which loads the robot in the Post Office world in Gazebo.
- **rur_navigation** - Handles the navigation and includes the **rur_navigation_post.launch** file.
- **butler_order.py** - The ROS Action Node that manages order execution.

---
## **Installation & Setup**
### **1. Clone the Repository**
```bash
cd ~/catkin_ws/src
git clone git@github.com:Mohamedaswer/Navigation.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### **2. Install Dependencies**
Ensure the following packages are installed:
```bash
sudo apt-get update
sudo apt-get install ros-noetic-navigation ros-noetic-move-base ros-noetic-map-server ros-noetic-amcl
```

---
## **Launching the Butler Robot**
### **1. Start the Simulation & Load World**
```bash
roslaunch rur_gazebo rur_postoffice.launch
```

### **2. Start Navigation**
```bash
roslaunch rur_navigation rur_navigation_post.launch
```

### **3. Start the Action Server**
```bash
rosrun rur_navigation butler_order.py
```

### **4. Send Orders to the Butler Robot**
#### **Basic Order (No Confirmation Needed):**
```bash
rostopic pub /butler/order std_msgs/String "data: 'table1'"
```
#### **Order with Confirmation & Timeout Handling:**
```bash
rostopic pub /butler/order std_msgs/String "data: 'table2 with confirmation'"
```
#### **Multiple Orders:**
```bash
rostopic pub /butler/order std_msgs/String "data: 'table1, table2, table3'"
```
#### **Cancel Order:**
```bash
rostopic pub /butler/cancel std_msgs/Bool "data: true"
```

---
## **Key Features**
### **1. Order Execution:**
- The robot moves from **home → kitchen → table → home**.
- Handles **single and multiple orders**.

### **2. Timeout Handling:**
- If **no one confirms receipt**, the robot **returns home** after a timeout.
- If the **kitchen does not confirm**, the robot **goes home**.

### **3. Cancellation Handling:**
- If **canceled while going to the kitchen**, the robot **returns home**.
- If **canceled while going to a table**, the robot **returns to the kitchen** first before going home.

### **4. Multiple Order Handling:**
- If no confirmation at one table, it **skips to the next table**.
- If an order is canceled mid-route, the robot **skips that table** and continues.
- After all deliveries, it **returns to the kitchen before home**.

---
## **Troubleshooting**
### **1. The robot does not move**
- Ensure `move_base` is running:  
  ```bash
  rosnode list | grep move_base
  ```
- Check if a goal is set:  
  ```bash
  rostopic echo /move_base/goal
  ```

### **2. Navigation Issues**
- Ensure the map is loaded correctly:  
  ```bash
  roslaunch rur_navigation map_server.launch
  ```
- Check localization:  
  ```bash
  rostopic echo /amcl_pose
  ```

---
## **Future Improvements**
- Implement **speech recognition** for voice-based order taking.
- Add **QR code scanning** for table detection.
- Integrate **multi-robot coordination** for better efficiency.

---
## **Contributors**
Developed by Mohamed Aswer. Contributions and improvements are welcome!

