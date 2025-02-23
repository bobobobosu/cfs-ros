from .cfs_service import run_node
def main():
    print("CFSClient is ready.")
    run_node()


# import sys
# import os
# sys.path.append(os.getcwd())
# os.environ["PYTHON_JULIACALL_HANDLE_SIGNALS"] = "yes"

# from juliacall import Main as jl

# os.environ["JULIA_NUM_THREADS"] = "12"
# import os
# import cfs_ros
# base_dir = os.path.dirname(os.path.abspath(cfs_ros.__file__))
# def main():
#     jl.include(os.path.join(base_dir, "julia", "test.jl"))
#     print("Test Julia")