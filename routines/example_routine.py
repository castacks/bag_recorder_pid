# import sys
# def pre_logger():
#     if sys.stdin.isatty():
#         choice = input("Yes or No: ")
#         with open("choice.txt", "w") as f:
#             f.write(choice)
#     else:
#         raise Exception("No terminal attached — cannot read user input")

def pre_logger(data: str="HELLO"):
    print(f"{data}!!!")
    print(f"{data}!!!")
    print(f"{data}!!!")
    print(f"{data}!!!")
    print(f"{data}!!!")
    print(f"{data}!!!")

def post_logger(data: str="GOODBYE :("):
    print(f"{data} 1")
    print(f"{data} 2")
    print(f"{data} 3")
    print(f"{data} 4")
    print(f"{data} 5")
    print(f"{data} 6")