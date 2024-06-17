# Robotica-Machinebouw-H
Handleiding en commentaar voor code Periode 2.4 Robotica in de Machinebouw.

**Arduino:**
Communicatie:
/Signaal
Start
StartCyclus
Stop

rosservice call /lampen "......"
WachtOpStart
InBedrijf
Fout
Storing

*BONUS*
Piep
Noodstop

Voor de installatie van de arduino met linux/ROS:
https://avans.sharepoint.com/:w:/s/AEI-ROS-Robot-Operating-System0841/EcvqOJ_AaCBBpeAs9AySBakBoot705q47F-Zxn9xOW7k-A?e=XZNjJb

**Camera Oak-D Pro:**
Voor de installatie van de DepthAI
https://avans.sharepoint.com/:w:/s/AEI-ROS-Robot-Operating-System0841/Ec2HpCpVXiBIrjsEPfQ13cEBkrA88vurGBB4vtmlMoKHyQ?e=5HJvUk

**Robot UFactory Lite 6:**
Voor installatie van de Robot
https://avans.sharepoint.com/:w:/s/AEI-ROS-Robot-Operating-System0841/EU5BhHd7MQ9OhqBt5MU01GYBWvODwn5Ksggk1Jgt9Bv_Rg?e=gdMnOT

**Camera**
rosservice call /lokalisatie ""


# Manipulator


Topic:
/move_group/display_planned_path                            Laat de geplande pad van de robot zien
/Signaal                                                    Ontvangt signaal van de verschillende knoppen
/Lampen                                                     Stuurt signaal naar arduino om lampen en/of geluid aan te sturen

Service:
/ufactory/vacuum_gripper_set                                Gripper open/sluiten met een 0 of 1

Node: 

