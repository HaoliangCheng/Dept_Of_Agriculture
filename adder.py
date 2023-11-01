from cnf import *
from ortools.sat.python import cp_model

def add_constraint_to_model(cnf, model, variables):
    for disj in cnf:
        conv_disj = [variables[lit] if not is_negated(lit) else
                     variables[lit[1]].Not() for lit in disj]
        model.AddBoolOr(conv_disj)

def create_variable(model, name, variables):
    variables[name] = model.NewBoolVar(name)

def create_variables(model, names, variables):
    #print("Creating %d variables" %(len(names)))
    for name in names: create_variable(model, name, variables)

# Half adder takes 2 inputs and produces two outputs (sum and c_out)
#   The c_out can be fed into another adder
# sum iff xor(a, b)
# c_out iff and(a, b)
def half_adder(model, a, b, sum, c_out, variables):
    sum_cnf = IFF(sum, XOR(a, b))
    add_constraint_to_model(sum_cnf, model, variables)

    c_out_cnf = IFF(c_out, AND(a, b))
    add_constraint_to_model(c_out_cnf, model, variables)

# Full adder takes 3 inputs and produces two outputs (sum and c_out)
#   The c_out can be fed into another adder
# sum iff xor(xor(a, b), c_in)
# c_out iff or(and(xor(a, b), c_in) and(a, b))
def full_adder(model, a, b, c_in, sum, c_out, variables):
    sum_cnf = IFF(sum, XOR(XOR(a, b), c_in))
    add_constraint_to_model(sum_cnf, model, variables)

    c_out_cnf = IFF(c_out, OR(AND(XOR(a, b), c_in), AND(a, b)))
    add_constraint_to_model(c_out_cnf, model, variables)

# Create an N-bit adder.
# a_bits, b_bits, and sum_bits are a list of ortool boolean variables,
#   starting with the low-order bit.
# a_bits and b_bits must be the same length, and sum_bits must be one longer
def n_bit_adder(model, a_bits, b_bits, sum_bits, variables):
    c_out = 'c_out0'
    create_variable(model, c_out, variables)
    half_adder(model, a_bits[0], b_bits[0], sum_bits[0], c_out, variables)
    for i in range(1, len(a_bits)):
        c_in = c_out
        c_out = 'c_out%d' %i
        create_variable(model, c_out, variables)
        full_adder(model, a_bits[i], b_bits[i], c_in, sum_bits[i], c_out,
                   variables)
    model.Add(variables[c_out] == variables[sum_bits[len(a_bits)]])

# Given the input bits as a binary list, with the low-order bit first,
#   find the sum using a logic-based n-bit adder.
# The inputs have the low-order bit first (e.g., [1, 1, 0] represents 3)
# Assumes a_bits and b_bits are same length
def input_output_adder(a_bit_values, b_bit_values):
    num_bits = len(a_bit_values)
    model = cp_model.CpModel()
    a_bits = ['a%d' %i for i in range(num_bits)]
    b_bits = ['b%d' %i for i in range(num_bits)]
    out_bits = ['sum%d' %i for i in range(num_bits+1)]

    variables = {}
    create_variables(model, (a_bits + b_bits + out_bits), variables)
    n_bit_adder(model, a_bits, b_bits, out_bits, variables)

    for i in range(num_bits):
        model.Add(variables[a_bits[i]] == a_bit_values[i])
        model.Add(variables[b_bits[i]] == b_bit_values[i])

    solver = cp_model.CpSolver()
    status = solver.Solve(model)
    #print("Branches: %f" %solver.NumBranches())
    #print("Wall time: %f" %solver.WallTime())
    return [solver.Value(variables[out_bit]) for out_bit in out_bits]

# This function takes the output bits (list of N bits, low-order bit first)
#   and returns a list of all possible N-1 bit inputs that would result in
#   that output
def output_input_adder(out_bit_values):
    num_bits = len(out_bit_values)-1
    model = cp_model.CpModel()
    a_bits = ['a%d' %i for i in range(num_bits)]
    b_bits = ['b%d' %i for i in range(num_bits)]
    out_bits = ['sum%d' %i for i in range(num_bits+1)]

    variables = {}
    create_variables(model, (a_bits + b_bits + out_bits), variables)
    n_bit_adder(model, a_bits, b_bits, out_bits, variables)

    for i in range(num_bits+1):
        model.Add(variables[out_bits[i]] == out_bit_values[i])

    solutions = []
    solver = cp_model.CpSolver()
    # BEGIN STUDENT CODE
    lista = [variables[ab] for ab in a_bits]
    listb = [variables[bb] for bb in b_bits]
    solution_collector = SolutionCollector(lista, listb)
    solver.SearchForAllSolutions(model, solution_collector)
    solutions = solution_collector.solutions
    
    # END STUDENT CODE

    return solutions

class SolutionCollector(cp_model.CpSolverSolutionCallback):
    solutions = []
    def __init__(self, a_bits, b_bits):
        cp_model.CpSolverSolutionCallback.__init__(self)
        # BEGIN STUDENT CODE
        self.solutions = []
        self.a_bits = a_bits
        self.b_bits = b_bits
        # END STUDENT CODE

    def OnSolutionCallback(self):
        # BEGIN STUDENT CODE
        sol = ([self.Value(v) for v in self.a_bits], 
               [self.Value(v) for v in self.b_bits])
        self.solutions.append(sol)
        # END STUDENT CODE
    

def convert_to_bits(value, nbits):
    if (value < 0 or value > 2**nbits-1):
        raise Exception("Value (%d) has to be between 0 and %d" %(value, 2**nbits-1))
    bits = [0]*nbits
    i = 0
    while value != 0:
        bits[i] = value%2
        value //= 2
        i += 1
    return bits

if __name__ == "__main__":
    import sys

    if len(sys.argv) < 3:
        print("Usage: adder.py addend1 addend2")
        exit()

    addends = [convert_to_bits(int(sys.argv[i]), 3) for i in [1,2]]
    print("%s + %s = %s" %(addends[0], addends[1], input_output_adder(addends[0], addends[1])))

