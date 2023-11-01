from cnf import *
from ortools.sat.python import cp_model

objects = ['Outlet', 'Rasp-Pi', 'Power-Board',
           'Arduino', 'Sensor-Board0', 'Sensor-Board1']
actuators = ['Fans', 'LEDs', 'Pump']
sensors = ['H-T0', 'Light0', 'Moisture0', 'H-T1', 'Light1', 'Moisture1',
           'Wlevel']
relations = ['working', 'connected', 'powered', 'signal', 'expected-result']

def powered(comp): return 'powered(%s)' %comp
def working(comp): return 'working(%s)' %comp
def connected(from_comp, to_comp):
    return 'connected(%s, %s)' %(from_comp, to_comp)
def signal(signal, component): return 'signal(%s, %s)' %(signal, component)
def rasp_pi_signal(the_signal): return signal(the_signal, 'Rasp-Pi')
def expected_result(actuator): return 'expected-result(%s)' %actuator

def create_relation(name, model, variables):
    variables[name] = model.NewBoolVar(name)

def create_relations(relations, model, variables):
    for relation in relations: create_relation(relation, model, variables)

def create_working_relations(model, variables):
    create_relations([working(comp) for comp in objects + actuators + sensors],
                     model, variables)

def create_connected_relations(model, variables):
    # BEGIN STUDENT CODE
    create_relation(connected("Outlet", "Power-Board"), model, variables)
    create_relation(connected("Outlet", "Rasp-Pi"), model, variables)
    create_relation(connected("Power-Board", "Fans"), model, variables)
    create_relation(connected("Power-Board", "Pump"), model, variables)
    create_relation(connected("Power-Board", "LEDs"), model, variables)
    create_relation(connected("Arduino", "Power-Board"), model, variables)
    create_relation(connected("Arduino", "Rasp-Pi"), model, variables)
    create_relation(connected("Rasp-Pi", "Arduino"), model, variables)
    create_relation(connected("Wlevel", "Arduino"), model, variables)
    create_relation(connected("Sensor-Board0", "Arduino"), model, variables)
    create_relation(connected("Sensor-Board1", "Arduino"), model, variables)
    create_relation(connected("H-T0", "Sensor-Board0"), model, variables)
    create_relation(connected("Light0", "Sensor-Board0"), model, variables)
    create_relation(connected("Moisture0", "Sensor-Board0"), model, variables)
    create_relation(connected("H-T1", "Sensor-Board1"), model, variables)
    create_relation(connected("Light1", "Sensor-Board1"), model, variables)
    create_relation(connected("Moisture1", "Sensor-Board1"), model, variables)
    
    
    create_relation(connected("H-T0", "Arduino"), model, variables)
    # END STUDENT CODE
    pass

def create_powered_relations(model, variables):
    # BEGIN STUDENT CODE
    create_relation(powered("Outlet"), model, variables)
    create_relation(powered("Rasp-Pi"), model, variables)
    create_relation(powered("Power-Board"), model, variables)
    create_relation(powered("Fans"), model, variables)
    create_relation(powered("LEDs"), model, variables)
    create_relation(powered("Pump"), model, variables)
    # END STUDENT CODE
    pass

def create_signal_relations(model, variables):
    # BEGIN STUDENT CODE
    create_relation(signal("H-T0", "H-T0"), model, variables)
    create_relation(signal("H-T0", "Sensor-Board0"), model, variables)
    create_relation(signal("H-T0", "Arduino"), model, variables)
    create_relation(signal("H-T0", "Rasp-Pi"), model, variables)
    create_relation(signal("Light0", "Light0"), model, variables)
    create_relation(signal("Light0", "Sensor-Board0"), model, variables)
    create_relation(signal("Light0", "Arduino"), model, variables)
    create_relation(signal("Light0", "Rasp-Pi"), model, variables)
    create_relation(signal("Moisture0", "Moisture0"), model, variables)
    create_relation(signal("Moisture0", "Sensor-Board0"), model, variables)
    create_relation(signal("Moisture0", "Arduino"), model, variables)
    create_relation(signal("Moisture0", "Rasp-Pi"), model, variables)
    create_relation(signal("H-T1", "H-T1"), model, variables)
    create_relation(signal("H-T1", "Sensor-Board1"), model, variables)
    create_relation(signal("H-T1", "Arduino"), model, variables)
    create_relation(signal("H-T1", "Rasp-Pi"), model, variables)
    create_relation(signal("Light1", "Light1"), model, variables)
    create_relation(signal("Light1", "Sensor-Board1"), model, variables)
    create_relation(signal("Light1", "Arduino"), model, variables)
    create_relation(signal("Light1", "Rasp-Pi"), model, variables)
    create_relation(signal("Moisture1", "Moisture1"), model, variables)
    create_relation(signal("Moisture1", "Sensor-Board1"), model, variables)
    create_relation(signal("Moisture1", "Arduino"), model, variables)
    create_relation(signal("Moisture1", "Rasp-Pi"), model, variables)
    create_relation(signal("Wlevel", "Wlevel"), model, variables)
    create_relation(signal("Wlevel", "Arduino"), model, variables)
    create_relation(signal("Wlevel", "Rasp-Pi"), model, variables)
    create_relation(signal("LEDs", "Rasp-Pi"), model, variables)
    create_relation(signal("LEDs", "Arduino"), model, variables)
    create_relation(signal("LEDs", "Power-Board"), model, variables)
    create_relation(signal("Fans", "Rasp-Pi"), model, variables)
    create_relation(signal("Fans", "Arduino"), model, variables)
    create_relation(signal("Fans", "Power-Board"), model, variables)
    create_relation(signal("Pump", "Rasp-Pi"), model, variables)
    create_relation(signal("Pump", "Arduino"), model, variables)
    create_relation(signal("Pump", "Power-Board"), model, variables)
    # END STUDENT CODE
    pass

def create_expected_result_relations(model, variables):
    # BEGIN STUDENT CODE
    create_relation(expected_result("Fans"), model, variables)
    create_relation(expected_result("LEDs"), model, variables)
    create_relation(expected_result("Pump"), model, variables)
    # END STUDENT CODE
    pass

def create_relation_variables(model):
    variables = {}
    create_working_relations(model, variables)
    create_connected_relations(model, variables)
    create_powered_relations(model, variables)
    create_signal_relations(model, variables)
    create_expected_result_relations(model, variables)
    return variables

def add_constraint_to_model(constraint, model, variables):
    for disj in (eval(constraint) if isinstance(constraint, str) else constraint):
        conv_disj = [variables[lit] if not is_negated(lit) else
                     variables[lit[1]].Not() for lit in disj]
        model.AddBoolOr(conv_disj)

def create_powered_constraint(from_comp, to_comp, model, variables):
    constraint = "IFF('%s', AND('%s', '%s'))" %(powered(to_comp),
                                                connected(from_comp, to_comp),
                                                working(from_comp))
    add_constraint_to_model(constraint, model, variables)

def create_powered_actuator_constraint(actuator, model, variables):
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', '%s'))))"
                  %(powered(actuator), connected('Power-Board', actuator),
                    powered('Power-Board'), working('Power-Board'),
                    signal(actuator, 'Power-Board')))
    add_constraint_to_model(constraint, model, variables)

def create_powered_constraints(model, variables):
    add_constraint_to_model(LIT(powered('Outlet')), model, variables)
    create_powered_constraint('Outlet', 'Rasp-Pi', model, variables)
    create_powered_constraint('Outlet', 'Power-Board', model, variables)
    for actuator in actuators:
        create_powered_actuator_constraint(actuator, model, variables)

def create_signal_constraint(from_comp, to_comp, model, variables):
    constraint = ("IFF('%s', AND('%s', AND('%s', '%s')))"
                  %(signal(from_comp, to_comp), 
                    connected(from_comp, to_comp),
                    working(from_comp),
                    signal(from_comp, from_comp)))
    add_constraint_to_model(constraint, model, variables)
def create_signal_constraints(model, variables):
    # BEGIN STUDENT CODE
    
    create_signal_constraint("H-T0", "Sensor-Board0", model, variables)
    create_signal_constraint("Light0", "Sensor-Board0", model, variables)
    create_signal_constraint("Moisture0", "Sensor-Board0", model, variables)
    create_signal_constraint("H-T1", "Sensor-Board1", model, variables)
    create_signal_constraint("Light1", "Sensor-Board1", model, variables)
    create_signal_constraint("Moisture1", "Sensor-Board1", model, variables)
    
    create_signal_constraint("Wlevel", "Arduino", model, variables)
    
    
    #create_signal_constraint("Sensor-Board0", "Arduino", model, variables)
    
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))"
                  %(signal("H-T0", "Arduino"), 
                    connected("H-T0", "Sensor-Board0"),
                    connected("Sensor-Board0", "Arduino"),
                    working("H-T0"),
                    working("Sensor-Board0"),
                    signal("H-T0", "Sensor-Board0")))
    add_constraint_to_model(constraint, model, variables)
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))"
                  %(signal("H-T1", "Arduino"), 
                    connected("H-T1", "Sensor-Board1"),
                    connected("Sensor-Board1", "Arduino"),
                    working("H-T1"),
                    working("Sensor-Board1"),
                    signal("H-T1", "Sensor-Board1")))
    add_constraint_to_model(constraint, model, variables)
    
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))"
                  %(signal("Moisture0", "Arduino"), 
                    connected("Moisture0", "Sensor-Board0"),
                    connected("Sensor-Board0", "Arduino"),
                    working("Moisture0"),
                    working("Sensor-Board0"),
                    signal("Moisture0", "Sensor-Board0")))
    add_constraint_to_model(constraint, model, variables)
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))"
                  %(signal("Moisture1", "Arduino"), 
                    connected("Moisture1", "Sensor-Board1"),
                    connected("Sensor-Board1", "Arduino"),
                    working("Moisture1"),
                    working("Sensor-Board1"),
                    signal("Moisture1", "Sensor-Board1")))
    add_constraint_to_model(constraint, model, variables)
    
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))"
                  %(signal("Light0", "Arduino"), 
                    connected("Light0", "Sensor-Board0"),
                    connected("Sensor-Board0", "Arduino"),
                    working("Light0"),
                    working("Sensor-Board0"),
                    signal("Light0", "Sensor-Board0")))
    add_constraint_to_model(constraint, model, variables)
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))"
                  %(signal("Light1", "Arduino"), 
                    connected("Light1", "Sensor-Board1"),
                    connected("Sensor-Board1", "Arduino"),
                    working("Light1"),
                    working("Sensor-Board1"),
                    signal("Light1", "Sensor-Board1")))
    add_constraint_to_model(constraint, model, variables)
    
    #===============================================
    
    # ACTUATOR TEST
    # Fans Arduino WORKING
    constraint = ("IFF('%s', AND('%s', AND('%s', '%s')))"
                  %(signal("Fans", "Arduino"), 
                    connected("Rasp-Pi", "Arduino"),
                    working("Rasp-Pi"),
                    signal("Fans", "Rasp-Pi")))
    add_constraint_to_model(constraint, model, variables)
    # LED Arduino WORKING
    constraint = ("IFF('%s', AND('%s', AND('%s', '%s')))"
                  %(signal("LEDs", "Arduino"), 
                    connected("Rasp-Pi", "Arduino"),
                    working("Rasp-Pi"),
                    signal("LEDs", "Rasp-Pi")))
    add_constraint_to_model(constraint, model, variables)
    # Pump
    constraint = ("IFF('%s', AND('%s', AND('%s', '%s')))"
                  %(signal("Pump", "Arduino"), 
                    connected("Rasp-Pi", "Arduino"),
                    working("Rasp-Pi"),
                    signal("Pump", "Rasp-Pi")))
    add_constraint_to_model(constraint, model, variables)
    
    #===============================================
    
    # Fans Power-Board
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))"
                  %(signal("Fans", "Power-Board"), 
                    connected("Rasp-Pi", "Arduino"),
                    connected("Arduino", "Power-Board"),
                    working("Rasp-Pi"),
                    working("Arduino"),
                    signal("Fans", "Rasp-Pi")))
    add_constraint_to_model(constraint, model, variables)
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))"
                  %(signal("LEDs", "Power-Board"), 
                    connected("Rasp-Pi", "Arduino"),
                    connected("Arduino", "Power-Board"),
                    working("Rasp-Pi"),
                    working("Arduino"),
                    signal("LEDs", "Rasp-Pi")))
    add_constraint_to_model(constraint, model, variables)
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))"
                  %(signal("Pump", "Power-Board"), 
                    connected("Rasp-Pi", "Arduino"),
                    connected("Arduino", "Power-Board"),
                    working("Rasp-Pi"),
                    working("Arduino"),
                    signal("Pump", "Rasp-Pi")))
    add_constraint_to_model(constraint, model, variables)
    
    #===============================================
    
    # sensor to Rasp-Pi
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))))"
                  %(signal("H-T0", "Rasp-Pi"), 
                    connected("H-T0", "Sensor-Board0"),#must
                    connected("Sensor-Board0", "Arduino"),#must
                    connected("Arduino", "Rasp-Pi"),#must
                    working("Sensor-Board0"),#must
                    working("Arduino"),#must
                    working("H-T0"),
                    signal("H-T0", "H-T0")))#must
    add_constraint_to_model(constraint, model, variables)
    
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))))"
                  %(signal("H-T1", "Rasp-Pi"), 
                    connected("H-T1", "Sensor-Board1"),#must
                    connected("Sensor-Board1", "Arduino"),#must
                    connected("Arduino", "Rasp-Pi"),#must
                    working("Sensor-Board1"),#must
                    working("Arduino"),#must
                    working("H-T1"),
                    signal("H-T1", "H-T1")))#must
    add_constraint_to_model(constraint, model, variables)
    
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))))"
                  %(signal("Light1", "Rasp-Pi"), 
                    connected("Light1", "Sensor-Board1"),#must
                    connected("Sensor-Board1", "Arduino"),#must
                    connected("Arduino", "Rasp-Pi"),#must
                    working("Sensor-Board1"),#must
                    working("Arduino"),#must
                    working("Light1"),
                    signal("Light1", "Light1")))#must
    add_constraint_to_model(constraint, model, variables)
    
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))))"
                  %(signal("Light0", "Rasp-Pi"), 
                    connected("Light0", "Sensor-Board0"),#must
                    connected("Sensor-Board0", "Arduino"),#must
                    connected("Arduino", "Rasp-Pi"),#must
                    working("Sensor-Board0"),#must
                    working("Arduino"),#must
                    working("Light0"),
                    signal("Light0", "Light0")))#must
    add_constraint_to_model(constraint, model, variables)
    
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))))"
                  %(signal("Moisture0", "Rasp-Pi"), 
                    connected("Moisture0", "Sensor-Board0"),#must
                    connected("Sensor-Board0", "Arduino"),#must
                    connected("Arduino", "Rasp-Pi"),#must
                    working("Sensor-Board0"),#must
                    working("Arduino"),#must
                    working("Moisture0"),
                    signal("Moisture0", "Moisture0")))#must
    add_constraint_to_model(constraint, model, variables)
    
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))"
                  %(signal("Wlevel", "Rasp-Pi"), 
                    connected("Wlevel", "Arduino"),#must
                    connected("Arduino", "Rasp-Pi"),#must
                    working("Arduino"),#must
                    working("Wlevel"),
                    signal("Wlevel", "Wlevel")))#must
    add_constraint_to_model(constraint, model, variables)
    
    constraint = ("IFF('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', AND('%s', '%s')))))))"
                  %(signal("Moisture1", "Rasp-Pi"), 
                    connected("Moisture1", "Sensor-Board1"),#must
                    connected("Sensor-Board1", "Arduino"),#must
                    connected("Arduino", "Rasp-Pi"),#must
                    working("Sensor-Board1"),#must
                    working("Arduino"),#must
                    working("Moisture1"),
                    signal("Moisture1", "Moisture1")))#must
    add_constraint_to_model(constraint, model, variables)
    
   
    # END STUDENT CODE
    pass

def create_sensor_generation_constraint(sensor, model, variables):
    # BEGIN STUDENT CODE
    constraint = ("IFF('%s', '%s')"
                  %(signal(sensor, sensor), 
                    working(sensor)))
    add_constraint_to_model(constraint, model, variables)
    # END STUDENT CODE
    pass
def create_sensor_generation_constraints(model, variables):
    # BEGIN STUDENT CODE
    create_sensor_generation_constraint("H-T0", model, variables)
    create_sensor_generation_constraint("Light0", model, variables)
    create_sensor_generation_constraint("Moisture0", model, variables)
    create_sensor_generation_constraint("H-T1", model, variables)
    create_sensor_generation_constraint("Light1", model, variables)
    create_sensor_generation_constraint("Moisture1", model, variables)
    create_sensor_generation_constraint("Wlevel", model, variables)
    # END STUDENT CODE
    pass

def create_expected_result_constraints(model, variables):
    # BEGIN STUDENT CODE
    
    # Fans OK
    constraint = ("IFF('%s', AND('%s', AND('%s', OR('%s', '%s'))))"
                   %(expected_result("Fans"), 
                     powered("Fans"),
                     working("Fans"),
                     signal("H-T0", "Rasp-Pi"),
                     signal("H-T1", "Rasp-Pi")))
    add_constraint_to_model(constraint, model, variables)
    constraint = ("IFF('%s', AND('%s', AND('%s', OR('%s', '%s'))))"
                   %(expected_result("LEDs"), 
                     powered("LEDs"),
                     working("LEDs"),
                     signal("Light0", "Rasp-Pi"),
                     signal("Light1", "Rasp-Pi")))
    add_constraint_to_model(constraint, model, variables)
    constraint = ("IFF('%s', AND('%s', AND('%s', OR('%s', OR('%s', '%s')))))"
                   %(expected_result("Pump"), 
                     powered("Pump"),
                     working("Pump"),
                     signal("Moisture0", "Rasp-Pi"),
                     signal("Moisture1", "Rasp-Pi"),
                     signal("Wlevel", "Rasp-Pi")))
    add_constraint_to_model(constraint, model, variables)
    # 2 more
    """
    constraint = "IFF('%s', AND('%s', AND('%s', OR('%s', '%s'))))"
                  %(expected-result("Fans"), 
                    powered("Fans"),
                    working("Fans"),
                    signal("H-T0", "Rasp-Pi"),
                    signal("H-T1", "Rasp-Pi"))
    print("IN")
    print(eval(constraint))
    print("OUT")
    add_constraint_to_model(constraint, model, variables)
    """
    # END STUDENT CODE
    pass

def create_constraints(model, variables):
    create_powered_constraints(model, variables)
    create_signal_constraints(model, variables)
    create_sensor_generation_constraints(model, variables)
    create_expected_result_constraints(model, variables)

def create_greenhouse_model():
    model = cp_model.CpModel()
    variables = create_relation_variables(model)
    create_constraints(model, variables)
    return (model, variables)
    
def collect_diagnosis(solver, variables):
    return set([var for var in variables
                if ((var.startswith('connected') or var.startswith('working')) and
                    solver.BooleanValue(variables[var]) == False)])

class DiagnosesCollector(cp_model.CpSolverSolutionCallback):
    def __init__(self, variables):
        cp_model.CpSolverSolutionCallback.__init__(self)
        # BEGIN STUDENT CODE
        # END STUDENT CODE

    def OnSolutionCallback(self):
        # Extract the connected and working relations that are False
        # BEGIN STUDENT CODE
        # END STUDENT CODE
        pass

def diagnose(observations):
    model, variables = create_greenhouse_model()
    add_constraint_to_model(observations, model, variables)

    collector = DiagnosesCollector(variables)
    diagnoses = []
    solver = cp_model.CpSolver()
    solver.SearchForAllSolutions(model, collector)
    # Remove all redundant diagnoses (those that are supersets
    #   of other diagnoses).
    # BEGIN STUDENT CODE
    # END STUDENT CODE

    return diagnoses
