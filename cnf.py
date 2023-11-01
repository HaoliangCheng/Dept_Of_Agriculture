# A CNF is a list of lists, where each sublist is an disjunct

def LIT(a): return [[a]]

# Each argument is either a CNF or a literal
def AND(a, b):
    a = _listify(a); b = _listify(b)
    cnf = a[:]
    for lit in b:
        if (not lit in cnf): cnf.append(lit)
    return _simplify(cnf)

def OR(a, b):
    a = _listify(a); b = _listify(b)
    cnf = []
    for disj1 in a:
        for disj2 in b:
            cnf.append(list(set(disj1 + disj2)))
    return _simplify(cnf)

def NOT(a):
    return _simplify(_crossproduct(_listify(a)))

def IMP(a, b): return OR(NOT(a), b)
def IFF(a, b): return AND(IMP(a, b), IMP(b, a))
def XOR(a, b): return AND(OR(a, b), NOT(AND(a,b )))

def is_negated(lit): return isinstance(lit, tuple) and lit[0] == '~'

def _listify(lit): return (lit if isinstance(lit, list) else [[lit]])

# Eliminate clauses that have A and ~A
def _simplify(cnf):
    return [disj for disj in cnf if 
            all(map(lambda lit: not _negate(lit) in disj, disj))]

def _negate(l):
    return (l[1] if is_negated(l) else ('~', l))

def _crossproduct(cnf):
    neg_disj = map(_negate, cnf[0])
    if (len(cnf) == 1):
        return [[nl] for nl in neg_disj]
    else:
        sub_cnf = _crossproduct(cnf[1:])
        cnf = []
        for lit in neg_disj:
            for disj in sub_cnf:
                if (not lit in disj):
                    l = list(set([lit]+disj))
                else: l = disj
                if (not l in cnf): cnf.append(l)
        return cnf
