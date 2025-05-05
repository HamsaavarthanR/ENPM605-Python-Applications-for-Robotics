#! /usr/bin/env/python3

# # ===================
# # ====== </1> ======
# # ===================
# def outer():
#     def inner():
#         print("I am a nested function!")
#     inner()  # Call inner function inside outer

# outer()  # Works, but no closure is formed

# # ===================
# # ====== </2> ======
# # ===================
# def outer(x):
#     def inner():
#         print(f"The value of x is {x}")  # # Closure captures 'x'
#     return inner  # Returning inner without executing it

# closure_function = outer(10)  # outer() finishes execution
# closure_function()  # Still remembers x = 10

# # ===================
# # ====== </3> ======
# # ===================
# def multiplier(factor):
#     def multiply(n):
#         return n * factor  # Closure captures 'factor'
#     return multiply  # Returns the inner function

# double = multiplier(2)
# triple = multiplier(3)

# print(double(5))  # Output: 10
# print(triple(5))  # Output: 15

# # ===================
# # ====== </4> ======
# # ===================
# def counter():
#     count = 0
#     def increment():
#         nonlocal count  # Closure captures 'count'
#         count += 1
#         return count
#     return increment  # Returns the inner function

# count_up = counter()
# print(count_up())  # Output: 1
# print(count_up())  # Output: 2
# print(count_up())  # Output: 3

# # ===================
# # ====== </5> ======
# # ===================
# def logger(name):
#     def log_message(message):
#         print(f"[{name}] {message}")  # Closure captures 'name'
#     return log_message

# app_logger = logger("APP")
# db_logger = logger("DB")

# app_logger("Starting application")  # Output: [APP] Starting application
# db_logger("Connecting to database")  # Output: [DB] Connecting to database

# # ===================
# # ====== </6> ======
# # ===================

# def sort_by_index(index):
#     def sort_func(item):
#         return item[index]  # Closure captures 'index'
#     return sort_func

# data = [(1, "apple"), (3, "banana"), (2, "cherry")]
# data.sort(key=sort_by_index(1))  # Sort by second element (name)

# print(data)  # Output: [(1, 'apple'), (3, 'banana'), (2, 'cherry')]

# # ===================
# # ====== </7> ======
# # ===================

# def decorate(func):
#     def wrapper():
#         print("~*" * 10)
#         func()  # Closure captures 'func'
#         print("~*" * 10)

#     return wrapper  # Returning the modified function


# def say_hello():
#     print("Hello")


# if __name__ == "__main__":
#     say_hello = decorate(say_hello)
#     say_hello()

# ===================
# ====== </8> ======
# ===================

# def decorate(func):
#     def wrapper():
#         print("~*" * 10)
#         func()  # Closure captures 'func'
#         print("~*" * 10)

#     return wrapper

# @decorate
# def say_hello(name=None):
#     print("Hello", name)

# if __name__ == "__main__":
#     say_hello("Jason") # call wrapper()


# ===================
# ====== </9> ======
# ===================
# def double_result(func):
#     def wrapper(*args, **kwargs):
#         result = func(*args, **kwargs)  # Call the original function
#         return result * 2  # Modify the return value
#     return wrapper

# @double_result
# def add(a, b):
#     return a + b

# print(add(3, 5))  # 16 (instead of 8)


# ===================
# ====== </10> ======
# ===================

# Exercise 1

# ===================
# ====== </11> ======
# ===================
# import functools


# def decorate(func):
#     @functools.wraps(func)  # Preserves function identity
#     def wrapper(*args, **kwargs):
#         print("~*" * 10)
#         func(*args, **kwargs)  # Closure captures 'func'
#         print("~*" * 10)

#     return wrapper


# @decorate
# def say_hello(name: str = "") -> None:
#     print("Hello" + name)

# if __name__ == "__main__":
#     print(say_hello.__name__)  # say_hello (instead of wrapper)
#     print(say_hello.__doc__)  # say_hello documentation
#     print(say_hello.__annotations__)  # say_hello annotations
#     say_hello()  # call wrapper()
#     say_hello.__wrapped__() # call say_hello()


# ===================
# ====== </12> ======
# ===================
# def decorate(func):
#     def wrapper(*args, **kwargs):
#         print("~*" * 10)
#         func(*args, **kwargs)  # Closure captures 'func'
#         print("~*" * 10)

#     return wrapper


# def normal_function(text: str = "") -> None:
#     print(text)


# if __name__ == "__main__":
#     decorated_function = decorate(
#         normal_function
#     )  # Only use the decorated version when needed

#     normal_function("original")  # Calls original function
#     decorated_function("decorated")  # Calls decorated version

# # ===================
# # ====== </13> ======
# # ===================

# import time
# import functools

# def timer(func):
#     @functools.wraps(func)
#     def wrapper(*args):
#         start_time = time.perf_counter()  # 1
#         func(*args)
#         end_time = time.perf_counter()  # 2
#         run_time = end_time - start_time  # 3
#         print(f"Finished {func.__name__!r} in {run_time:.4f} secs")

#     return wrapper

# @timer
# def waste_some_time(num_times):
#     for _ in range(num_times):
#         sum([i**2 for i in range(10000)])
    
# waste_some_time(1000)

# # ===================
# # ====== </14> ======
# # ===================
# import time
# import functools

# def slow_down(func):
#     """Sleep 1 second before calling the function"""

#     @functools.wraps(func)
#     def wrapper(*args):
#         time.sleep(1)
#         return func(*args)

#     return wrapper


# @slow_down
# def countdown(from_number):
#     if from_number < 1:
#         print("🚀🚀 Liftoff!")
#     else:
#         print(from_number)
#         countdown(from_number - 1)

# countdown(3)

# # ===================
# # ====== </15> ======
# # ===================
# class Robot:
#     """
#     A class representing a robot with basic movement capabilities.

#     This class provides a blueprint for creating robot objects with specific
#     physical attributes and implements basic movement functionality.

#     Attributes:
#         name (str): The name of the robot.
#         weight (float): The weight of the robot in kilograms.
#         width (float): The width of the robot in centimeters.
#     """

#     def __init__(self, name: str, weight: float, width: float) -> None:
#         """
#         Initialize a new Robot instance.

#         Args:
#             name (str): The name to assign to the robot.
#             weight (float): The weight of the robot in kilograms.
#             width (float): The width of the robot in centimeters.
#         """
#         self.name = name
#         self.weight = weight
#         self.width = width

#     def walk_forward(self) -> None:
#         """
#         Make the robot walk forward.

#         Prints a message indicating that the robot is walking forward.

#         Returns:
#             None
#         """
#         print(f"{self.name} is walking forward")

#     def walk_backward(self) -> None:
#         """
#         Make the robot walk backward.

#         Prints a message indicating that the robot is walking backward.

#         Returns:
#             None
#         """
#         print(f"{self.name} is walking backward")

#     def jump(self) -> None:
#         """
#         Make the robot jump.

#         Prints a message indicating that the robot is jumping.

#         Returns:
#             None
#         """
#         print(f"{self.name} is jumping")

# # ===================
# # ====== </16> ======
# # ===================

# if __name__ == '__main__':
#     nimble = Robot('Nimble', 65.0, 20.0)
#     print(nimble.name, nimble.weight, nimble.width)
#     nimble.walk_forward()
#     nimble.walk_backward()
#     nimble.jump()
    
#     dash = Robot('Dash', 65.0, 20.0)
#     print(dash.name, dash.weight, dash.width)
#     dash.walk_forward()
#     dash.walk_backward()
#     dash.jump()

# # ===================
# # ====== </17> ======
# # ===================

# a = 1
# b = 2
# print(a.__add__(2))  # a + 2
# print(a.__sub__(2))  # a - 2
# print(a.__mul__(2))  # a * 2
# print(a.__truediv__(2))  # a / 2
# print(a.__floordiv__(2))  # a // 2
# print(a.__mod__(2))  # a % 2
# print(a.__pow__(2))  # a ** 2
# print(a.__gt__(b))  # a > b
# print(a.__ge__(b))  # a >= b
# print(a.__lt__(b))  # a < b
# print(a.__le__(b))  # a <= b
# print(a.__eq__(b))  # a == b