Open folder  ELIDEK_617

run main.py

#   main.py      #
select direction of earthquake
select number of parameters sets  (file Parameters.txt)
select number of earthquakes      (folder earthquakes)

#    Bridge      #
initialize and construct bridge model
create the appropriate node lists
create the neseccary functions (eg. update, restore, solve etc)

#    orchestrator      #
set time function (select cosim DT,bridge and vehicle analysis dt, global time)
select vehicle speed  (included xml for a 5-car train ) (vehicle xml _0=initial definition of train configuration)
select parameters for cosim ( number of iter(implicit/explicit), tolerances)

#   vehicle solver     #
vehicle solver is called via vehicle.dll ()
API functions are defined in  API_vehicle.h


