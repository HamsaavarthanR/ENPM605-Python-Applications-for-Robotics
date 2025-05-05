import sys

# ----------------
# </> 1
# ----------------

# print("Welcome", "to ENPM", 809, "E")
# print("Welcome", "to ENPM", 809, "E", sep="***")

# ----------------
# </> 2
# ----------------

# # standard assignment
# name = "Guido van Rossum"
# # chained assignments
# x = y = 10
# print(x, y)
# # multiple assignments
# name, age, role = "Guido van Rossum", 64, "BDFL Emeritus"
# print(name, age, role)

# ----------------
# </> 3
# ----------------
# a = 10

# print(type(a))  # <class 'int'>
# print(type(100.5))  # <class 'float'>
# print(type(print))  # <class 'builtin_function_or_method'>

# ----------------
# </> 4
# ----------------
# a = 10
# b = 10.5
# c = "hello"

# print(id(a))  # 139869549249040
# print(id(b))  # 139869548203760
# print(id(c))  # 139869548514160
# print(id(10))  # guess the output
# print(id(10.5))  # guess the output
# print(id("hello"))  # guess the output


# ----------------
# </> 5
# ----------------
# x = 10
# print(type(x))      # <class 'int'>

# y = 10.5
# print(type(y))      # <class 'float'>

# z = 'hello'
# print(type(z))      # <class 'str'>


# ----------------
# </> 6
# ----------------
# x = "hello"
# print(type(x))  # <class 'str'>
# x = 10.5
# print(type(x))  # <class 'float'>

# ----------------
# </> 7
# ----------------
# x = 1

# if x == 1:
#     print("x is 1")

# print("outside code block")

# ----------------
# </> 8
# ----------------
# x = 1

# if x == 1:
#     print("x is 1")
# else:
#     print("x is not 1")

# print("outside code block")

# ----------------
# </> 9
# ----------------
# x = 5

# if x == 1:
#     print("x is 1")
# elif x == 2:
#     print("x is 2")
# elif x == 3:
#     print("x is 3")
# else:
#     print("x is not 1, 2, or 3")

# ----------------
# </> 10
# ----------------
# x = "hello"
# if x:
#     print("x is True")
# else:
#     print("x is False")

# ----------------

# print(bool(0))  # False
# print(bool(1))  # True
# print(bool(-2))  # True
# print(bool(""))  # False
# print(bool(" "))  # True

# x = "hello"
# print(bool(x))  # True

# ----------------
# </> 11
# ----------------
# a = 1
# b = 2
# print(a == b)  # False
# print(a != b)  # True
# print(a > b)  # False
# print(a < b)  # True
# print(a >= b)  # False
# print(a <= b)  # True

# ----------------
# </> 12
# ----------------
# a = "hello"
# b = 0
# print(bool(a and b)) # ??
# print(bool(a or b)) # ??
# print(bool(not (a or b))) # ??


# ----------------
# </> 13
# ----------------
# x = "hello"
# print("h" in x)  # ??
# print("he" in x)  # ??
# print("O" in x)  # ??

# ----------------
# </> 14
# ----------------
# a = "hello"
# b = 2
# print(a is b)  # False
# print(a is not b)  # True

# ----------------
# </> 15
# ----------------
# a = None
# if a is None:
#     print("a is None")

# print(type(None))  # <class 'NoneType'>

# ----------------
# </> 16
# ----------------
# number = 123
# print(type(number))  # <class 'int'>
# number_str = str(number)
# print(type(number_str))  # <class 'str'>

# ----------------
# </> 17
# ----------------
# print("This string\nhas\nbeen\nsplit")
# print("1\t2\t3\t4")
# print("He said: \"I'm here!\"")
# print('He said: "I\'m here!"')
# print('''He said: "I'm here!"''')
# print("""He said: "I'm here!" """) # note the space before the ending triple quotes
# # print("C:\Users\tony\notes.txt") # TODO: Fix this

# ----------------
# </> 18
# ----------------
# name = "John"
# age = 30
# print("His name is %s and he is %d years old." % (name, age))

# ----------------
# </> 19
# ----------------
# name = "Alice"
# age = 25
# print("Her name is {} and she is {} years old.".format(name, age))
# # With positional arguments
# print("Her name is {1} and she is {0} years old.".format(age, name))

# ----------------
# With keyword arguments
# print("Her name is {name} and she is {age} years old.".format(name="Alice", age=25))

# ----------------
# </> 20
# ----------------
# name = "Alice"
# age = 25
# print(f"Her name is {name} and she is {age} years old.")  # note the f before the string
# print(f"Her name is {name} and she is {age} years old.")  # note the F before the string

# ----------------
# </> 21
# ----------------
# first_name = "John"
# last_name = "Doe"
# print(first_name + " " + last_name)  # John Doe
# ----------------
# words = ["Hello", "world"]
# sentence = " ".join(words)
# print(sentence)  # Hello world
# ----------------
# first_name = "John"
# last_name = "Doe"
# full_name = "%s %s" % (first_name, last_name)
# print(full_name)  # John Doe
# full_name = "{} {}".format(first_name, last_name)
# print(full_name)  # John Doe
# full_name = f"{first_name} {last_name}"
# print(full_name)  # John Doe

# ----------------
# </> 22
# ----------------
# print(len("hello"))  # 5
# ----------------
# print(str(3), type(str(3)))  # 3 <class 'str'>
# print(str(3 + 4), type(str(3 + 4)))  # 7 <class 'str'>


# ----------------
# </> 23
# ----------------
# print("hello".capitalize())  # Hello
# print("HeLLo".swapcase())  # hEllO

# ----------------
# </> 24
# ----------------
# print("hello"[0])  # h
# greeting = "hello"
# print(greeting[1])  # e
# ----------------
# print("hello"[-5])  # h
# greeting = "hello"
# print(greeting[-4])  # e


# ----------------
# </> 25
# ----------------
# print("hello"[5])
# greeting = "hello"
# greeting[0] = "c"
# print(greeting)

# ----------------
# </> 26
# ----------------
# greeting = "hello"

# # Slicing with positive indices
# print(greeting[0:3])  # ??
# print(greeting[:3])  # ??
# print(greeting[:5])  # ??
# print(greeting[:])  # ??

# # Slicing with negative indices
# print(greeting[-5:])  # ??
# print(greeting[-5:-2])  # ??

# # Slicing with positive and negative indices
# print(greeting[-5:2])  # ??
# print(greeting[-5:4])  # ??

# ----------------
# </> 27
# ----------------
# quote = "Learn Python, be happy!"
# #----------------
# print(quote[::]) # Learn Python, be happy!
# print(quote[::1]) # Learn Python, be happy!
# #----------------
# print(quote[::2])  # LanPto,b ap!
# print(quote[::3])  # LrPh,eay
# #----------------
# print(quote[:8:-1])  # !yppah eb ,noh

# ----------------
# </> 28
# ----------------
# quote = "Learn Python, be happy!"

# ----------------
# </> 29
# ----------------
# a = "hello"
# b = "hello"
# c = "h" + "ello"
# d = "he" + "llo"
# d = "".join(["h", "e", "l", "l", "o"])
# e = "{0}{1}".format("h", "ello")

# print(a is b)  # True
# print(c is d)  # False
# print(a is d)  # False
# print(a is e)  # False

# ----------------
# </> 30
# ----------------
# a = "hello"
# b = "hello"
# c = sys.intern("h" + "ello")
# d = sys.intern("he" + "llo")
# d = sys.intern("".join(["h", "e", "l", "l", "o"]))
# e = sys.intern("{0}{1}".format("h", "ello"))

# print(a is b)  # True
# print(c is d)  # True
# print(a is d)  # True
# print(a is e)  # True

# ----------------
# </> 31
# ----------------
# print(10 // 3)  # ??
# print(10 // -3)  # ??

# ----------------
# </> 32
# ----------------
# print(int(3.5))             # 3

# print(int('3'))             # 3      

# print(int('101011', 2))     # 43

# ----------------
# </> 33
# ----------------
# a, b = 20, 20
# print(a is b)   # True

# a, b = -5, -5
# print(a is b)   # True

# a, b = 200000000000000, 200000000000000
# print(a is b)   # True

# ----------------
# </> 34
# ----------------
# print(float("  3.5  \n"))  # 3.5
# print(float('3'))           # 3.0
# print(float(3))             # 3.0

# ----------------
# </> 35
# ----------------
# text = "HelloWorld"
# # second_half = ??
# print(second_half)