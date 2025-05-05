# # ==================
# # ====== </1> ======
# # ==================

# def greet_user(user_name):
#     """
#     Prints a greeting message for the specified user.

#     Args:
#         user_name (str): The name of the user.
#     """
#     print(f"Hello, {user_name}!")


# if __name__ == "__main__":
#     greet_user("John")
#     # print(greet_user.__doc__)

# # ==================
# # ====== </3> ======
# # ==================

# # function definition
# def add(a, b):
#     return a + b


# # function call
# result = add(1, 2)
# print(f'{result=}') # result=3
# # print(a, b)

# # ==================
# # ====== </4> ======
# # ==================

# def greet():
#     print("Hello!")


# print(globals()["greet"])

# print(greet.__code__.co_code)

# # ==================
# # ====== </5> ======
# # ==================

# def greet():
#     print("Hello!")


# def greet():
#     print("Hi!")


# greet()

# # ==================
# # ====== </6> ======
# # ==================

# def add_numbers(a: None, b: dict):
#     return a + b

# # ==================
# # ====== </7> ======
# # ==================

# from typing import List, Tuple, Dict

# def process_data(data: List[Tuple[int, str]]) -> Dict[str, int]:
#     return {name: value for value, name in data}

# # Example usage
# data_sample = [(1, "apple"), (2, "banana")]
# result = process_data(data_sample)  # {'apple': 1, 'banana': 2}
# print(result)

# # ==================
# # ====== </8> ======
# # ==================
# from typing import Union

# def parse_input(value: str) -> Union[int, float, str]:
#     try:
#         return int(value)
#     except ValueError:
#         try:
#             return float(value)
#         except ValueError:
#             return value

# # Example usage
# print(parse_input("42"))     # 42
# print(parse_input("3.14"))   # 3.14
# print(parse_input("hello"))  # "hello"


# # ==================
# # ====== </9> ======
# # ==================
# from typing import Optional, List

# def find_index(lst: List[str], item: str) -> Optional[int]:
#     return lst.index(item) if item in lst else None

# # Example usage
# names = ["John", "Jane", "Marc"]
# print(find_index(names, "Mark"))  # None
# print(find_index(names, "Marc"))  # 2

# # ===================
# # ====== </10> ======
# # ===================
# from typing import TypeVar, List

# T = TypeVar("T")  # Generic type

# def reverse_list(lst: List[T]) -> List[T]:
#     return lst[::-1]

# # Example usage
# print(reverse_list([1, 2, 3]))        # [3, 2, 1]
# print(reverse_list(["a", "b", "c"]))  # ['c', 'b', 'a']
# print(reverse_list([["a"], ["b"], ["c"]]))  # [['c'], ['b'], ['a']]

# # ===================
# # ====== </11> ======
# # ===================
# from typing import TypeVar, List

# A = TypeVar("A", str, int)  # Only str or int


# def reverse_list(items: List[A]) -> List[A]:
#     return items[::-1]


# # Example usage
# print(reverse_list(["apple", "banana"]))  # ['banana', 'apple']
# print(reverse_list([10, 20, 30]))  # [30, 20, 10]

# # ===================
# # ====== </12> ======
# # ===================
# from typing import TypeVar, List, Union

# B = TypeVar("B", float, int, List[str])  # Must be float, int, or List[str]


# def process_value(value: B) -> Union[float, int, str]:
#     if isinstance(value, (int, float)):
#         return value * 2  # Double the number
#     elif isinstance(value, list):
#         return ", ".join(value)  # Join list into a string
#     else:
#         raise ValueError("Invalid type provided")


# # Example usage
# print(process_value(10))  # 20
# print(process_value(5.5))  # 11.0
# print(process_value(["apple", "banana", "cherry"]))  # "apple, banana, cherry"

# # ===================
# # ====== </13> ======
# # ===================
# def display_info(name, age):
#     print(f"{name=}, {age=}")

# # Correct usage
# display_info("Alice", 30)  # name='Alice', age=30

# # Incorrect usage - values are swapped
# display_info(30, "Alice")  # name=30, age='Alice'

# # ===================
# # ====== </14> ======
# # ===================
# def display_info(name, age=35):
#     print(f"{name=}, {age=}")


# # Calling function without specifying the age parameter
# display_info("Bob")  # name='Bob', age=35

# display_info("Alice", 30)  # name='Alice', age=30


# # ===================
# # ====== </15> ======
# # ===================
# def greet(message="Hello", name):
#     print(f"{message}, {name}!")

# # ===================
# # ====== </16> ======
# # ===================
# def create_user(name, age, is_admin):
#     print(f"{name=}, {age=}, {is_admin=}")

# # Calling the function using keyword arguments
# create_user(age=30, is_admin=True, name="Alice") # Output: name='Alice', age=30, is_admin=True


# # ===================
# # ====== </17> ======
# # ===================
# def introduce(name, age, city):
#     print(f"My name is {name}, I am {age} years old, and I live in {city}.")

# # Correct usage: Positional arguments first, followed by keyword arguments
# introduce("Alice", city="New York", age=25)  # Works fine

# # Incorrect usage: Mixing positional and keyword arguments incorrectly
# introduce(name="Alice", 25, city="New York")  # SyntaxError: positional argument follows keyword argument

# # ===================
# # ====== </18> ======
# # ===================
# from typing import Union

# def sum_numbers(*args: Union[int, float]) -> Union[int, float]:
#     print(type(args))
#     for item in args:
#         print(item)
#     return sum(args)

# # Function calls with different numbers of arguments
# print(sum_numbers(2, 4, 6))        # Output: 12
# print(sum_numbers(1, 2, 3, 4, 5))  # Output: 15
# print(sum_numbers())               # Output: 0

# # ===================
# # ====== </19> ======
# # ===================
# from typing import Any

# def print_user_info(**kwargs: Any) -> None:
#     print(type(kwargs))
#     for key, value in kwargs.items():
#         print(f"{key}: {value}")

# # Function calls with different keyword arguments
# print_user_info(name="Alice", age=30, city="New York")
# # name: Alice
# # age: 30
# # city: New York

# print_user_info(language="Python", version=3.10, is_dynamic=True)
# # language: Python
# # version: 3.10
# # is_dynamic: True

# # ===================
# # ====== </20> ======
# # ===================
# from typing import Any


# def display_info(*args: Any, **kwargs: Any) -> None:
#     print("Positional arguments:", args)
#     print("Keyword arguments:", kwargs)


# # Example usage
# display_info(1, "hello", True, name="Alice", age=25)
# # Positional arguments: (1, 'hello', True)
# # Keyword arguments: {'name': 'Alice', 'age': 25}

# # ===================
# # ====== </21> ======
# # ===================
# def print_everything(*args, **kwargs):
#     for arg in args:
#         print(arg)
#     for key, value in kwargs.items():
#         print(f"{key}: {value}")

# # Example usage
# print_everything(1, 'Apple', username='john_doe', age=25)

# # ===================
# # ====== </22> ======
# # ===================
# def display_info(name: str, age: int, city: str) -> None:
#     print(f"Name: {name}, Age: {age}, City: {city}")


# # Using * for positional argument unpacking
# user_data = ("Alice", 30, "New York")
# display_info(*user_data)
# # display_info(user_data[0], user_data[1], user_data[2])
# user_data = ["Alice", 30, "New York"]
# display_info(*user_data)
# # display_info(user_data[0], user_data[1], user_data[2])

# # ===================
# # ====== </23> ======
# # ===================
# def print_everything(*args, **kwargs):
#     for arg in args:
#         print(arg)
#     for key, value in kwargs.items():
#         print(f"{key}: {value}")

# # Example usage
# print_everything(1, 'Apple', username='john_doe', age=25)

# # ===================
# # ====== </24> ======
# # ===================
# def display_info(name, age, city):
#     print(f"Name: {name}, Age: {age}, City: {city}")


# # Using * for positional argument unpacking
# user_data = ("Alice", 30, "New York")
# display_info(*user_data)
# user_data = ["Alice", 30, "New York"]
# display_info(*user_data)

# # ===================
# # ====== </25> ======
# # ===================
# def display_info(name, age, city):
#     print(f"Name: {name}, Age: {age}, City: {city}")

# # Using ** for keyword argument unpacking
# user_info = {"name": "Bob", "age": 25, "city": "San Francisco"}
# display_info(**user_info)

# # ===================
# # ====== </26> ======
# # ===================
# def my_function():
#     x = 10  # Local variable
#     print(x)  # Python looks for x in the local scope first

# my_function()  # Output: 10

# # ===================
# # ====== </27> ======
# # ===================
# def outer_function():
#     print("Outer function is executing.")

#     def inner_function():
#         print("Inner function is executing.")

#     inner_function()  # Calling the inner function

# outer_function()
# inner_function() # NameError: name 'inner_function' is not defined.

# # ===================
# # ====== </28> ======
# # ===================
# def outer():
#     y = 20  # Enclosing scope variable

#     def inner():
#         print(y)  # Python searches in the local scope of 'inner', then 'outer'

#     inner()


# outer()  # Output: 20

# # ===================
# # ====== </29> ======
# # ===================
# def outer_function():
#     enclosing_var = "outer function"

#     def inner_function():
#         nonlocal enclosing_var
#         enclosing_var = "inner function"
#         print(enclosing_var)  # "inner function"

#     inner_function()
#     print(enclosing_var)  # "inner function"

# outer_function()

# # ===================
# # ====== </30> ======
# # ===================
# message = "Hello, World!"  # Global variable

# def greet():
#     print(message)  # Accessing the global variable

# greet()  # Hello, World!

# # ===================
# # ====== </31> ======
# # ===================
# count = 0  # Global variable

# def increment():
#     global count  # We are modifying the global variable
#     count += 1

# increment()
# print(count)  # Output: 1

# # ===================
# # ====== </32> ======
# # ===================
# fruits = ["apple", "banana"]

# def add_to_list(a):
#     fruits.append(a)

# add_to_list("orange")
# print(fruits) # ["apple", "banana", "orange]

# # ===================
# # ====== </33> ======
# # ===================
# x = 10  # Global variable


# def example():
#     x = 5  # Local variable (does not affect global x)
#     print("Inside function:", x)  # Inside function: 5


# example()
# print("Outside function:", x)  # Outside function: 10

# # ===================
# # ====== </34> ======
# # ===================
# print(dir(__builtins__))

# # ===================
# # ====== </35> ======
# # ===================
# a = 1 # global
# b = 1 # global

# def outer_function():
#     a = 10 # enclosing
#     b = 10 # enclosing

#     def inner_function():
#         global b
#         nonlocal a
#         b = 100
#         a = 100
#         print(f"inner: {a=} {b=}")
#     inner_function()
#     print(f"outer: {a=} {b=}")

# outer_function()
# print(f"global: {a=} {b=}")

# # ===================
# # ====== </36> ======
# # ===================
# def modify_number(x):
#     x += 10  # Creates a new local variable x, does not modify the original
#     print("Inside function:", x)  # Inside function: 15


# num = 5
# modify_number(num)
# print("Outside function:", num)  # Outside function: 5

# # ===================
# # ====== </37> ======
# # ===================
# def modify_list(lst):
#     lst.append(4)  # Modifies the original list
#     print("Inside function:", lst) # [1, 2, 3, 4]

# numbers = [1, 2, 3]
# modify_list(numbers)
# print("Outside function:", numbers) # [1, 2, 3, 4]

# # ===================
# # ====== </38> ======
# # ===================
# from copy import copy


# def modify_list(lst):
#     lst = copy(lst)  # Create a shallow copy of the list
#     lst.append(4)
#     print("Inside function:", lst)  # [1, 2, 3, 4]


# numbers = [1, 2, 3]
# modify_list(numbers)
# print("Outside function:", numbers)  # [1, 2, 3]

# # ============================
# # ====== </39> & </40>  ======
# # ============================
def edit_inputs(fruits, animals, age):
    fruits.append("cherry")
    fruits[0] = "mango"
    fruits = ["quince", "pear"]
    animals = ["elephant"]
    age += 1


fruits = ["apple", "banana"]
animals = ["bear", "tiger"]
age = 40

print(fruits, animals, age)  # ['apple', 'banana'] ['bear', 'tiger'] 40
edit_inputs(fruits, animals, age)
print(fruits, animals, age)  # ??
