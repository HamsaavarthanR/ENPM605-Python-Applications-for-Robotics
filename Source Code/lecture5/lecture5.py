# ===================
# ====== </1> ======
# ===================
def my_function():
    pass


print(callable(my_function))  # True
print(callable(42))  # False
print(callable([1, 2, 3]))  # False


# # ===================
# # ====== </2> ======
# # ===================
# def factorial(number):
#     # Initialize the factorial result to 1 (the identity value for multiplication)
#     factorial = 1
#     # Multiply the result by each number from 1 to the specified number
#     for i in range(1, number + 1):
#         factorial *= i
#     return factorial


# if __name__ == "__main__":
#     number = 5
#     print(f"The factorial of {number} is {factorial(number)}")


# # ===================
# # ====== </3> ======
# # ===================
# def celsius_to_fahrenheit(celsius):
#     return (celsius * 9 / 5) + 32


# def display_temperature(city, temp_c):
#     temp_f = celsius_to_fahrenheit(temp_c)
#     print(f"Temperature in {city}: {temp_c}°C / {temp_f}°F")


# if __name__ == "__main__":
#     display_temperature("New York", 21)
#     display_temperature("Paris", 18)


# # ===================
# # ====== </4> ======
# # ===================

# lambda x: x * 2  # Doubles a value
# lambda x, y: x + y  # Adds two values
# lambda s: s.strip().upper()  # Strips and uppercases a string

# print((lambda x: x * 2)(2))  # 4
# print((lambda x, y: x + y)(2, 3))  # 5
# print((lambda s: s.strip().upper())("hello   "))  # HELLO


# # ===================
# # ====== </5> ======
# # ===================
# # Check if a number is prime
# def is_prime(n):
#     if n < 2:
#         return False
#     for i in range(2, int(n**0.5) + 1):
#         if n % i == 0:
#             return False
#     return True


# print(is_prime(7))  # Expected output: True
# print(is_prime(10))  # Expected output: False

# # Not recommended for readability:
# is_prime = lambda n: all(n % i != 0 for i in range(2, int(n**0.5) + 1)) and n > 1
# print(is_prime(7))  # Expected output: True
# print(is_prime(10))  # Expected output: False

# # ===================
# # ====== </5> ======
# # ===================

# # Double a number
# def add(x, y):
#     return x + y


# print(add(1, 3))  # 4

# print("=" * 20)

# add = lambda x, y: x + y
# print(add(1, 3))  # 4

# # ===================
# # ====== </6> ======
# # ===================

# import dis

# # Disassemble lambda expression
# dis.dis(lambda x: x * 2)

# print("=" * 20)


# # Disassemble standard function
# def double(x):
#     return x * 2


# dis.dis(double)

# # ===================
# # ====== </7> ======
# # ===================
# students = [
#     {"name": "Alice", "grade": 85},
#     {"name": "Bob", "grade": 92},
#     {"name": "Charlie", "grade": 78},
# ]

# lowest_grade_student = min(students, key=lambda s: s["grade"])

# print(lowest_grade_student)  # {'name': 'Charlie', 'grade': 78}

# # ===================
# # ====== </8> ======
# # ===================

# students = [
#     {"name": "Alice", "scores": {"math": 85, "english": 78}},
#     {"name": "Bob", "scores": {"math": 92, "english": 88}},
#     {"name": "Charlie", "scores": {"math": 78, "english": 90}}
# ]

# # Your code here

# print(sorted_students)
# # Output: [{'name': 'Bob', ...}, {'name': 'Alice', ...}, {'name': 'Charlie', ...}]

# # ===================
# # ====== </9> ======
# # ===================
# concat_strings = # Your lambda function here

# print(concat_strings("Hello", "World"))  # Expected output: "Hello World"
# print(concat_strings("Lambda", "Functions"))  # Expected output: "Lambda Functions"

# # ====================
# # ====== </10> ======
# # ====================

# def greet(name):
#     return f"Hello, {name}!"

# my_greeting = greet  # Function assigned to variable
# print(my_greeting("Alice"))  # Prints: Hello, Alice!

# # ====================
# # ====== </11> ======
# # ====================

# def add(x, y):
#     return x + y

# def subtract(x, y):
#     return x - y

# operations = [add, subtract]
# print(operations[0](5, 3))  # 8
# print(operations[1](5, 3))  # 2

# print("=" * 20)

# function_list = [str.lower, str.upper, str.capitalize]
# text = "Python Programming"
# for func in function_list:
#     print(func(text))  # Applies each function to the text

# # ====================
# # ====== </12> ======
# # ====================


# def apply_twice(func, value):
#     """Apply a function twice to a value"""
#     return func(func(value))


# def add_five(x):
#     return x + 5


# def subtract(x):
#     return x - 1


# result = apply_twice(add_five, 10)  # 10 + 5 + 5 = 20
# print(result)  # 20
# result = apply_twice(subtract, 10)  # 10 - 1 - 1 = 8
# print(result)  # 8

# # ====================
# # ====== </13> ======
# # ====================
# def make_multiplier(factor):
#     return lambda x: x * factor


# double = make_multiplier(2)
# triple = make_multiplier(3)

# print(double(4))  # 8
# print(triple(4))  # 12

# # ====================
# # ====== </14> ======
# # ====================

# numbers = [1, 2, 3, 4, 5]
# squared = map(lambda x: x**2, numbers)
# print(list(squared))  # [1, 4, 9, 16, 25]

# print("=" * 20)

# str_nums = ["1", "2", "3", "4"]
# int_nums = map(int, str_nums)
# print(list(int_nums))  # [1, 2, 3, 4]

# print("=" * 20)


# def celsius_to_fahrenheit(c):
#     return (c * 9 / 5) + 32


# temperatures_c = [0, 10, 20, 30]
# temperatures_f = map(celsius_to_fahrenheit, temperatures_c)
# print(list(temperatures_f))  # [32.0, 50.0, 68.0, 86.0]

# # ====================
# # ====== </15> ======
# # ====================

# list1 = [1, 2, 3]
# list2 = [10, 20, 30]
# sums = map(lambda x, y: x + y, list1, list2)
# print(list(sums))  # [11, 22, 33]

# print("=" * 20)

# names = ["alice", "bob", "charlie"]
# ages = [25, 30, 35]
# cities = ["new york", "london", "paris"]

# # Create formatted strings using all three lists
# formatted = map(
#     lambda n, a, c: f"{n.title()} is {a} years old and lives in {c.title()}",
#     names,
#     ages,
#     cities,
# )
# print(list(formatted))

# # ====================
# # ====== </16> ======
# # ====================

# Exercise 3

# # ====================
# # ====== </17> ======
# # ====================
# numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
# even_numbers = filter(lambda x: x % 2 == 0, numbers)
# print(list(even_numbers))  # [2, 4, 6, 8, 10]

# print("=" * 20)

# values = ["apple", "", "banana", None, "cherry", 0]
# truth_values = filter(None, values)
# print(list(truth_values))  # ['apple', 'banana', 'cherry']

# # ====================
# # ====== </18> ======
# # ====================

# Exercise 4

# # ====================
# # ====== </19> ======
# # ====================

# from functools import reduce

# numbers = [1, 2, 3]
# sum_result = reduce(lambda x, y: x + y, numbers)
# print(sum_result)  # 6

# print("=" * 20)

# numbers = [1, 2, 3]
# sum_result = reduce(lambda x, y: x + y, numbers, 10)
# print(sum_result)  # 16

# # ====================
# # ====== </20> ======
# # ====================

# numbers = [5, 2, 9, 1, 5, 6]
# sorted_numbers = sorted(numbers)
# print(sorted_numbers)  # [1, 2, 5, 5, 6, 9]

# print("=" * 20)

# words = ["apple", "banana", "cherry", "date"]
# sorted_words = sorted(words, key=len)
# print(sorted_words)  # ['date', 'apple', 'banana', 'cherry']


# print("=" * 20)

# pairs = [(1, "b"), (3, "a"), (2, "c")]
# sorted_pairs = sorted(pairs, key=lambda x: x[1])
# print(sorted_pairs)  # [(3, 'a'), (1, 'b'), (2, 'c')]


# # ====================
# # ====== </21> ======
# # ====================
# from functools import partial


# def multiply(x, y):
#     return x * y


# double = partial(multiply, y=2)
# print(double(4))  # 8
# print(double(10))  # 20

# print("=" * 20)


# def greet(greeting, name):
#     return f"{greeting}, {name}!"


# say_hello = partial(greet, "Hello")
# say_hi = partial(greet, "Hi")

# print(say_hello("Alice"))  # Hello, Alice!
# print(say_hi("Bob"))  # Hi, Bob!

# # ====================
# # ====== </22> ======
# # ====================
# from functools import partial

# def format_number(number, precision, suffix):
#     return f"{number:.{precision}f}{suffix}"

# format_percentage = partial(format_number, precision=1, suffix='%')
# format_dollars = partial(format_number, precision=2, suffix='$')

# print(format_percentage(75.2))  # 75.2%
# print(format_dollars(45.8))     # 45.80$

# print("=" * 20)


# def query_db(host, port, database, query):
#     return f"Connecting to {host}:{port}/{database} to execute: {query}"

# # Create a function with fixed connection details
# local_query = partial(query_db, host='localhost', port=5432, database='myapp')

# print(local_query(query="SELECT * FROM users"))

# # ====================
# # ====== </23> ======
# # ====================
# Exercise 5

# # ====================
# # ====== </24> ======
# # ====================

# def double(x):
#     return x * 2

# def increment(x):
#     return x + 1

# # Manual composition
# result = double(increment(5))  # (5 + 1) * 2 = 12
# print(result)

# # ====================
# # ====== </25> ======
# # ====================

# def double(x):
#     return x * 2

# def increment(x):
#     return x + 1

# def compose(f, g):
#     """Compose two functions: f(g(x))"""
#     return lambda x: f(g(x))

# # Usage
# increment_then_double = compose(double, increment)
# print(increment_then_double(5))  # 12

# # Multiple compositions
# def square(x):
#     return x * x

# # Compose three functions
# increment_then_double_then_square = compose(square, compose(double, increment))
# print(increment_then_double_then_square(5))  # ((5 + 1) * 2)^2 = 144

# # ====================
# # ====== </26> ======
# # ====================


# def compose(*functions):
#     def composed_func(x):
#         for f in reversed(functions):
#             x = f(x)
#         return x

#     return composed_func


# # Define some basic functions
# def increment(x):
#     return x + 1


# def square(x):
#     return x * x


# def half(x):
#     return x / 2


# # Usage
# composed_number_function = compose(half, square, increment)
# print(composed_number_function(3))  # 3 -> 4 -> 16 -> 8.0


# # ====================
# # ====== </27> ======
# # ====================

# def factorial(n):
#     # Base case
#     if n == 0 or n == 1:
#         return 1
#     # Recursive case
#     return n * factorial(n - 1)

# print(factorial(5))  # 5 * 4 * 3 * 2 * 1 = 120

# print("=" * 20)

# def fibonacci(n):
#     # Base cases
#     if n <= 0:
#         return 0
#     if n == 1:
#         return 1
#     # Recursive case
#     return fibonacci(n-1) + fibonacci(n-2)

# # Print first 6 Fibonacci numbers
# for i in range(6):
#     print(fibonacci(i), end=" ")  # 0, 1, 1, 2, 3, 5