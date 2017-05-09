## SWITCHER DEMO ##

require 'orocos'
require 'readline'
require 'vizkit'
require 'rock/bundle'

include Orocos
Orocos.initialize

Orocos.conf.load_dir('../../../bundles/exoter/config/orogen/')

Orocos.run 'locomotion_control::Task' => 'locomotion_control',
           'locomotion_switcher::Task' => 'locomotion_switcher',
           'wheelwalking_control::Task' => 'wheel_walking_control',
           'controldev::JoystickTask'=>'joystick' do

  ## SETUP ##

  # setup platform_driver
    puts "Setting up platform_driver"
    platform_driver = Orocos.name_service.get 'platform_driver'
    Orocos.conf.apply(platform_driver, ['default'], :override => true)
    platform_driver.configure
    puts "done"

  # setup read dispatcher
    puts "Setting up reading joint_dispatcher"
    read_joint_dispatcher = Orocos.name_service.get 'read_joint_dispatcher'
    Orocos.conf.apply(read_joint_dispatcher, ['reading'], :override => true)
    read_joint_dispatcher.configure
    puts "done"

  # setup the commanding dispatcher
    puts "Setting up commanding joint_dispatcher"
    command_joint_dispatcher = Orocos.name_service.get 'command_joint_dispatcher'
    Orocos.conf.apply(command_joint_dispatcher, ['commanding'], :override => true)
    command_joint_dispatcher.configure
    puts "done"

  # setup exoter ptu_control
    puts "Setting up ptu_control"
    ptu_control = Orocos.name_service.get 'ptu_control'
    Orocos.conf.apply(ptu_control, ['default'], :override => true)
    ptu_control.configure
    puts "done"

  # setup exoter wheel_walking_control
    puts "Setting up wheel_walking_control"
    wheel_walking_control = Orocos.name_service.get 'wheel_walking_control'
    Orocos.conf.apply(wheel_walking_control, ['default'])
    wheel_walking_control.configure
    puts "done"

  # setup exoter locomotion_control
    puts "Setting up locomotion_control"
    locomotion_control = Orocos.name_service.get 'locomotion_control'
    Orocos.conf.apply(locomotion_control, ['default'], :override => true)
    locomotion_control.configure
    puts "done"

  # setup exoter locomotion_switcher
    puts "Setting up locomotion_switcher"
    locomotion_switcher = Orocos.name_service.get 'locomotion_switcher'
    locomotion_switcher.configure
    puts "done"

  # setup joystick
    puts "Setting up joystick"
    joystick = Orocos.name_service.get 'joystick'
    Orocos.conf.apply(joystick, ['default', 'logitech_gamepad'], :override => true)
    joystick.configure
    puts "done"


  ## CONNECTING PORTS ##

    Orocos.log_all_ports
    puts "Connecting ports"

  # Connecting joystick outputs
    joystick.motion_command.connect_to                   locomotion_switcher.joystick_motion_command
    joystick.raw_command.connect_to                      locomotion_switcher.joystick_command

  # Connecting locomotion_switcher outputs
    locomotion_switcher.joints_commands.connect_to       command_joint_dispatcher.joints_commands
    locomotion_switcher.ww_readings.connect_to           wheel_walking_control.joint_readings
    locomotion_switcher.ww_joystick_command.connect_to   wheel_walking_control.joystick_commands
    locomotion_switcher.lc_readings.connect_to           locomotion_control.joints_readings
    locomotion_switcher.lc_motion_command.connect_to     locomotion_control.motion_command
    locomotion_switcher.bema_command.connect_to          locomotion_control.bema_command
    locomotion_switcher.walking_command_front.connect_to locomotion_control.walking_command_front
    locomotion_switcher.walking_command_rear.connect_to  locomotion_control.walking_command_rear

  # Connecting locomotion_control to locomotion_switcher
    locomotion_control.joints_commands.connect_to        locomotion_switcher.lc_commands
    locomotion_control.bema_joints.connect_to            locomotion_switcher.bema_joints

  # Connecting wheel_walking_control to locomotion switcher
    wheel_walking_control.joint_commands.connect_to      locomotion_switcher.ww_commands   

  # Connect ports: platform_driver to read_joint_dispatcher
    platform_driver.joints_readings.connect_to           read_joint_dispatcher.joints_readings

  # Connect ports: read_joint_dispatcher to locomotion switcher
    read_joint_dispatcher.joints_samples.connect_to      locomotion_switcher.joints_readings

  # Connect ports: command_joint_dispatcher to platform_driver
    command_joint_dispatcher.motors_commands.connect_to  platform_driver.joints_commands

  # Connect ports: read_joint_dispatcher to ptu_control
    read_joint_dispatcher.ptu_samples.connect_to         ptu_control.ptu_samples

  # Connect ports: ptu_control to command_joint_dispatcher
    ptu_control.ptu_commands_out.connect_to              command_joint_dispatcher.ptu_commands
    puts "done"


  ## STARTING TASKS ##
    locomotion_control.start
    locomotion_switcher.start
    wheel_walking_control.start
    joystick.start
    platform_driver.start
    read_joint_dispatcher.start
    command_joint_dispatcher.start
    ptu_control.start

    Readline::readline("Press ENTER to exit\n") do
    end
end
