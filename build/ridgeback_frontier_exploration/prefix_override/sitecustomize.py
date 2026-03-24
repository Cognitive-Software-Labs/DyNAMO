import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/czika-stefania/Documents/DyNAMO/DyNAMO/install/ridgeback_frontier_exploration'
