require 'orocos'
require 'readline'

include Orocos
Orocos.initialize

Orocos.conf.load_dir('../../../bundles/exoter/config/orogen/')

Orocos.run 'locomotion_control::Task' => 'locomotion_control',
           'locomotion_switcher::Task' => 'locomotion_switcher',
           'simulation_vrep::Task' => 'simulation_vrep',
           'wheelwalking_control::Task' => 'wheel_walking_control',
           'controldev::JoystickTask'=>'joystick' do
  
  locomotion_control = Orocos.name_service.get 'locomotion_control'
  Orocos.conf.apply(locomotion_control, ['default'], :override => true)
  locomotion_control.configure

  puts "LC Configured"

  simulation_vrep = Orocos.name_service.get 'simulation_vrep'
  simulation_vrep.configure

  locomotion_switcher = Orocos.name_service.get 'locomotion_switcher'
  locomotion_switcher.configure

  # setup exoter wheel_walking_control
  puts "Setting up wheel_walking_control"
  wheel_walking_control = Orocos.name_service.get 'wheel_walking_control'
  Orocos.conf.apply(wheel_walking_control, ['default'])
  wheel_walking_control.configure
  puts "done"

  joystick = Orocos.name_service.get 'joystick'
  Orocos.conf.apply(joystick, ['default', 'logitech_gamepad'], :override => true)
  joystick.configure
  puts "joystick configured"


  joystick.motion_command.connect_to                   locomotion_switcher.joystick_motion_command
  joystick.raw_command.connect_to                      locomotion_switcher.joystick_command
        
  locomotion_switcher.joints_commands.connect_to       simulation_vrep.joints_commands
  locomotion_switcher.ww_readings.connect_to           wheel_walking_control.joint_readings
  locomotion_switcher.ww_joystick_command.connect_to   wheel_walking_control.joystick_commands
  locomotion_switcher.lc_readings.connect_to           locomotion_control.joints_readings
  locomotion_switcher.lc_motion_command.connect_to     locomotion_control.motion_command
  locomotion_switcher.bema_command.connect_to          locomotion_control.bema_command
  locomotion_switcher.walking_command_front.connect_to locomotion_control.walking_command_front
  locomotion_switcher.walking_command_rear.connect_to  locomotion_control.walking_command_rear

  simulation_vrep.joints_readings.connect_to           locomotion_switcher.joints_readings
  simulation_vrep.motors_readings.connect_to           locomotion_switcher.motors_readings

  locomotion_control.joints_commands.connect_to        locomotion_switcher.lc_commands
  locomotion_control.bema_joints.connect_to            locomotion_switcher.bema_joints

  wheel_walking_control.joint_commands.connect_to      locomotion_switcher.ww_commands   
    
  
  simulation_vrep.start
  locomotion_control.start
  locomotion_switcher.start
  wheel_walking_control.start
  joystick.start

  Readline::readline("Press ENTER to exit\n")

end
