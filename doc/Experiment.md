# ICRA 2023 Experiment

## Description of the Experiment

The robot does a pick and place task and the user do a LEGO composition.

- The robot has to pick some LEGO pieces and bring them to the operator. The pieces will be stacked in blocks. (e.g. 3 cubes together)
- Some pieces will be in the availability of the operator, to speed up the assembly.
- The operator has to take the pieces, divide them and then assemble them according to a reference scheme.

## Experiment with Alexa Conversation

### Event 1 - Unforeseen Obstacle (Conversation)

- The robot has to pick a piece and bring it to the operator.
- In the place area there is another piece and the robot cannot place the piece.
- The robot asks the operator to move the obstacle.

        R: "There is an obstacle where I have to place the object, can you move it?" | Ask API: 'ObstacleDetected_ObjectMoved'

        Option 1 - U: "Yes, I'll move it right away." | API: 'ObstacleDetected_ObjectMoved'

        Option 2 - U: "No, I'm busy".
        R: "can I put it somewhere else?" | Ask API: 'PutObject'

            Option 1: U: "Put it here / in the box (point-at)" | API: 'PutObject'
            R: "ok"

            Option 2: U: "No, wait" | API: 'WaitForCommand'
            R: "Ok, I'll wait for your command"

### Event 2 - Operator Proximity (Conversation)

- The robot has to do something near the operator. (e.g. insert a particular piece)
- The robot asks the operator to move to have space ("I have to slow down too much / stop, can you move?")
- The operator moves away or tell to the robot to place the block in another location or do it late.

        R: "I have to come there, can you move back?" | Ask API: 'UserMoved'
        R: "I'm slowing down too much because I'm coming towards you, can you move back?" | Ask API: 'UserMoved'

        Option 1 : U: "Yes" -> Moves Back
        R: "Thanks" | API: 'UserMoved'

        Option 2: U: "Put it in 30 seconds" | API: 'DelayObject'
        R: "Ok, I'll try again in 30 seconds"

        Option 3: U: "No, I can't"
        R: "Ok, can I put the object somewhere else?" | Ask API: 'PutObject'

            Option 1: U: "You can put it here (point-at)" | API: 'PutObject'
            R: "Ok"

            Option 2: U: "No / wait until I finish." | API: 'WaitForCommand'
            R: "Ok, I'll wait for your command"

## Experiment without Alexa Conversation

### Event 1 - Unforeseen Obstacle (Alexa Standard)

        R: "There is an obstacle where I have to place the object"

        Option 1 - U: "Alexa, I moved it right away." | Intent: 'MovedTheObject'
        R: "Ok"

        Option 2 - U: "Alexa, put the object here (point-at)" | Intent: 'PutObject'
        R: "Ok"

### Event 2 - Operator Proximity (Alexa Standard)

        R: "I have to come there, can you move back?"

        Option 1 : U: "Alexa, I moved back" | Intent: 'UserMoved'
        R: "Thanks"

        Option 2: U: "Alexa, I can't move" | Intent: 'UserCantMove'
        R: "Ok, can I put the object somewhere else?"

            Option 1: U: "Alexa, put it here (point-at)" | Intent: 'PutObject'
            R: "Ok"

            Option 2: U: "Alexa, wait" | Intent: 'Wait'
            R: "Ok, I'll wait for your command"

            Option 3: U: "Alexa, I moved" | Intent: 'CanGo'

## Run the Experiment

- Launch Robot Drivers:

        roslaunch ur_rtde_controller rtde_controller.launch enable_gripper:=true

- Launch Manipulator Planner:

        roslaunch manipulator_planner ur10e_planner.launch sim:=false

- Launch Trajectory Scaling:

        source ~/.../orocos_ws/devel/setup.bash
        roslaunch trajectory_scaling ur10e_scaling.launch optitrack:=true human_radius:=0.20
        roslaunch trajectory_scaling ur10e_sim_scaling.launch human_radius:=0.20

- Launch VRPN Client for Optitrack (already launched in Trajectory Scaling Launchfile with `optitrack:=true`):

        roslaunch vrpn_client_ros sample.launch server:=192.168.2.50

- Launch Alexa (Normal or Conversation):

        Alexa Standard:

                conda activate alexa_env
                "./ngrok http 5000" -> set the skill endpoint to the ngrok url
                roslaunch alexa_voice_control skill_backend.launch

        Alexa Conversation:

                conda activate alexa_conversation_env
                roslaunch alexa_conversation skill_backend.launch
                set the skill endpoint to the ngrok url (add "/api/SkillBackend")

- Launch Mediapipe Gesture Recognition (+ Point-At Area):

        conda activate multimodal_fusion
        roslaunch mediapipe_gesture_recognition stream_node.launch realsense:=true
        roslaunch mediapipe_gesture_recognition recognition_node.launch point_area:=true

- Launch Multimodal Fusion:

        conda activate multimodal_fusion
        roslaunch multimodal_fusion multimodal_fusion.launch

- Launch the Experiment:
  Normal → `conversation:=false` | Conversation → `conversation:=true`

        conda activate multimodal_fusion
        roslaunch multimodal_fusion experiment.launch enable_gripper:=true conversation:=true
