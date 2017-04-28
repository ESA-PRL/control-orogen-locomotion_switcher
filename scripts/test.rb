require 'orocos'
require 'readline'

include Orocos
Orocos.initialize

Orocos.conf.load_dir('../../../bundles/exoter/config/orogen/')

Orocos.run 'locomotion_control::Task' => 'locomotion_control',
           'locomotion_switcher::Task' => 'locomotion_switcher',
           'simulation_vrep::Task' => 'simulation_vrep' do
  
  locomotion_control = Orocos.name_service.get 'locomotion_control'
  Orocos.conf.apply(locomotion_control, ['default'], :override => true)
  locomotion_control.configure

  puts "LC Configured"

  simulation_vrep = Orocos.name_service.get 'simulation_vrep'
  simulation_vrep.configure

  locomotion_switcher = Orocos.name_service.get 'locomotion_switcher'
  locomotion_switcher.configure

  simulation_vrep.joints_readings.connect_to          locomotion_control.joints_readings
  simulation_vrep.joints_readings.connect_to          locomotion_switcher.joints_readings
  locomotion_control.joints_commands.connect_to       locomotion_switcher.lc_commands
  locomotion_switcher.joints_commands.connect_to       simulation_vrep.joints_commands

  
  simulation_vrep.start
  locomotion_control.start
  locomotion_switcher.start

  Readline::readline("Press ENTER to exit\n")

end
