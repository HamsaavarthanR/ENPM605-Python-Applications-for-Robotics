from pprint import pprint


# ----------------
# </> 1
# ----------------
# question = "What is blue and smells like red paint?"

# for character in question:
#     print(character)

# ----------------
# </> 2
# ----------------
# fruits = ["apple", "banana"]
# result = fruits.append("orange") # add orange to the list
# print(result) # ??
# print(fruits)  # ['apple', 'banana', 'orange']

# ----------------
# </> 3
# ----------------
# fruit = "apple"
# upper_fruit = fruit.upper()  # Returns a new string, original is unchanged
# print(fruit)  # apple
# print(upper_fruit)  # APPLE

# ----------------
# </> 4
# ----------------
# fruits1 = ['apple', 'banana', 'cherry']
# fruits2 = ['cherry', 'banana', 'apple']
# fruits3 = ['apple', 'banana', 'cherry']
# print(fruits1 == fruits2)    # False
# print(fruits1 is fruits2)    # False
# print(fruits1 == fruits3)    # True
# print(fruits1 is fruits3)    # False

# ----------------
# </> 5
# ----------------
# a = ["apple", "banana", 1]
# ----------------
# ----------------
# a = list()  # []
# print(a)
# a = list("apple")  # ['a', 'p', 'p', 'l', 'e']
# print(a)
# a = list(["apple", "banana"])  # ['apple', 'banana']
# print(a)
# ----------------
# ----------------
# # create an empty list
# square_num = []
# # use a loop to fill out the list
# for num in range(1, 5):
#     square_num.append(num * num)

# print(square_num)  # [1, 4, 9, 16]
# ----------------
# ----------------
# # create and fill out a new list at the same time
# square_num = [number * number for number in range(1, 5)]
# print(square_num)  # [1, 4, 9, 16]

# ----------------
# </> 6
# ----------------
# quote = 'The answer lies in the heart of battle!'
# vowel_list = [char for char in quote if char in 'aeiouy']
# print(vowel_list)

# ----------------
# </> 7
# ----------------
# numbers = [3, 4, -5, 45, -55]
# numbers = [n if n > 0 else 0 for n in numbers]
# print(numbers)

# ----------------
# </> 8
# ----------------
# matrix = [
#     [0, 0, 0, 0],
#     [1, 1, 1, 1],
#     [2, 2, 2, 2],
#     [3, 3, 3, 3],
# ]

# # with list comprehension
# flattened_matrix = [num for row in matrix for num in row]
# print(flattened_matrix)

# # with a for loop
# flattened_matrix = []
# for row in matrix:
#     for num in row:
#         flattened_matrix.append(num)

# print(flattened_matrix)

# ----------------
# </> 9
# ----------------
# fruits = ['apple', 'banana', 'cherry', 'pear', 'kiwi']
# print(fruits[1])        # banana
# print(fruits[-2])       # pear
# print(fruits[-4:-1])    # ['banana', 'cherry', 'pear']


# ----------------
# </> 10
# ----------------
# fruits = ['apple', 'banana', 'cherry', 'pear', 'kiwi']
# fruits[0] = 'orange'    # ['orange', 'banana', 'cherry', 'pear', 'kiwi']

# fruits[1:3] = []        # ['orange', 'pear', 'kiwi']
# fruits[:] = ['orange', 'orange', 'orange'] # ['orange', 'orange', 'orange']
# fruits[:] = ['apple']   # ['apple']
# fruits[:] = 'apple'     # ['a', 'p', 'p', 'l', 'e']

# print()

# --- Exercise
# fruits = ['apple', 'banana', 'cherry', 'pear', 'kiwi']
# # write code here
# print(fruits)   # ['apple', 'banana', 'cherry', [], 'kiwi']

# ----------------
# </> 11
# ----------------
# fruits = ["apple", "banana", "cherry", "kiwi", "apple"]
# del fruits[0]
# print(fruits)  # ['banana', 'cherry', 'kiwi', 'apple']
# del fruits[1:]
# print(fruits)  # ['banana']
# # ----------------
# # ----------------
# fruits = ['apple', 'banana', 'cherry', 'kiwi', 'apple']
# del fruits
# print(fruits) # NameError: name 'fruits' is not defined

# ----------------
# </> 12
# ----------------
# fruits = ['apple', 'banana']
# fruits = ['kiwi'] + fruits  # ['kiwi', 'apple', 'banana']
# fruits = fruits + ['cherry']  # ['kiwi', 'apple', 'banana', 'cherry']
# # # ----------------
# # # ----------------
# fruits = ['apple']
# fruits += ['banana'] # ['apple', 'banana']
# fruits += 'kiwi' # ['apple', 'banana', 'k', 'i', 'w', 'i']

# print()

# ----------------
# </> 13
# ----------------
# fruits = ["apple"]
# fruits.append("kiwi")  # ['apple', 'kiwi']
# print(fruits)
# print("=" * 10)
# # # # ----------------
# # # # ----------------
# fruits = ["apple"]
# fruits.extend(["kiwi"])  # ['apple', 'kiwi']
# print(fruits)
# print("=" * 10)
# # # # ----------------
# # # # ----------------
# fruits = ["apple"]
# fruits.insert(0, "kiwi")  # ['kiwi', 'apple']
# print(fruits)
# print("=" * 10)
# # # # ----------------
# # # # ----------------
# fruits = ["fig", "kiwi", "fig"]
# fruits.remove("fig")  # ['kiwi', 'fig']
# print(fruits)
# print("=" * 10)
# # # # ----------------
# # # # ----------------
# fruits = ["apple", "kiwi"]
# print(fruits.pop(1))  # kiwi
# print(fruits)
# print("=" * 10)
# # # # ----------------
# # # # ----------------
# fruits = ["apple", "kiwi"]
# fruits.clear()  # []
# print(fruits)
# print("=" * 10)

# ----------------
# </> 14
# ----------------
# string_list = ['a', ['bb', ['ccc', '123'], 'ee', 'ff'], 'g']
# # --- Exercise
# print() # 123
# print() # 2

# ----------------
# </> 15
# ----------------
# fruits1 = ['apple']
# fruits2 = fruits1
# print(fruits1 is fruits2)   # True
# # ----------------
# # ----------------
# fruits1 = ['apple']
# fruits2 = fruits1
# fruits1.append('kiwi')
# fruits2.append('orange')
# print(fruits1)   # ['apple', 'kiwi', 'orange']
# print(fruits2)   # ['apple', 'kiwi', 'orange']

# ----------------
# </> 16
# ----------------
# from copy import copy

# num1 = [1, [2]]
# num2 = copy(num1)               # SHALLOW COPY
# print(f'{num1=}', f'{num2=}')   # num1=[1, [2]] num2=[1, [2]]
# print(num1 is num2)             # False

# num1.append(3)
# print(f'{num1=}', f'{num2=}')   # num1=[1, [2], 3] num2=[1, [2]]

# num1[0] = 4
# print(f'{num1=}', f'{num2=}')   # num1=[4, [2], 3] num2=[1, [2]]

# num1[1].append(100)
# print(f'{num1=}', f'{num2=}')   # num1=[4, [2, 100], 3] num2=[1, [2, 100]]

# ----------------
# </> 17
# ----------------
# from copy import deepcopy

# num1 = [1, [2]]
# num2 = deepcopy(num1)           # DEEP COPY
# print(f'{num1=}', f'{num2=}')   # num1=[1, [2]] num2=[1, [2]]
# print(num1 is num2)             # False

# num1.append(3)
# print(f'{num1=}', f'{num2=}')   # num1=[1, [2], 3] num2=[1, [2]]

# num1[0] = 4
# print(f'{num1=}', f'{num2=}')   # num1=[4, [2], 3] num2=[1, [2]]

# num1[1].append(100)
# print(f'{num1=}', f'{num2=}')   # num1=[4, [2, 100], 3] num2=[1, [2]]

# ----------------
# </> 18
# ----------------
# numbers = [1, 2, 3, 1, 1]
# print(numbers.count(1))  # 3
# # ----------------
# # ----------------
# numbers = [2, 3, 2, 1, 3]
# print(numbers.index(3))  # 1
# # ----------------
# # ----------------
# numbers = [2, 3, 2, 1, 3]
# numbers.reverse()
# print(numbers)  # [3, 1, 2, 3, 2]
# # ----------------
# # ----------------
# numbers = ["a", "d", "ccc", "bb"]
# numbers.sort()
# print(numbers)  # ['a', 'bb', 'ccc', 'd']
# # ----------------
# # ----------------
# numbers = ["a", "d", "ccc", "bb"]
# new_numbers = sorted(numbers)
# print(numbers)  # ['a', 'd', 'ccc', 'bb']
# print(new_numbers)  # ['a', 'bb', 'ccc', 'd']


# ----------------
# </> 19
# ----------------
# a = [0, 1, [2, 3]]
# print(a.pop().pop())
# # ----------------
# # ----------------
# a = ['1', '2', '3']
# print(a[1:])
# # ----------------
# # ----------------
# x = ["a", "b"]
# y = "cde"
# print(x.extend(y))
# # ----------------
# # ----------------
# fruits = ["apple", "banana", "cherry", "kiwi", "mango"]
# output = [x for x in fruits if "a" in x]
# print(output)
# # ----------------
# # ----------------
# fruits = ["apple", "banana", "cherry", "kiwi", "mango"]
# # ----------------
# # ----------------
# numbers = ["a", "d", "ccc", "bb"]


# ----------------
# </> 20
# ----------------
# fruits = ('apple', 'kiwi', 'orange')
# print(type(fruits))  # <class 'tuple'>
# # ----------------
# # ----------------
# fruits = 'apple', 'kiwi', 'orange'

# # singleton
# fruits = 'apple'
# print(type(fruits))  # <class 'str'>

# fruits = 'apple',
# print(type(fruits))  # <class 'tuple'>
# # ----------------
# # ----------------
# fruits = tuple()
# print(fruits)
# fruits = tuple(['apple', 'kiwi', 'orange'])
# print(fruits)

# ----------------
# </> 21
# ----------------
# fruits = ("apple", "kiwi")
# fruit1, fruit2 = fruits
# print(fruit1, fruit2)  # apple kiwi

# fruits = ("apple", "kiwi", "orange")
# fruit1, fruit2 = fruits  # too many values to unpack (expected 2)
# fruit1, fruit2, fruit3, fruit4 = (
#     fruits  # not enough values to unpack (expected 4, got 3)
# )
# # ----------------
# # ----------------
# fruits = ('apple', 'kiwi', 'orange')
# fruit1, _, fruit3 = fruits
# # ----------------
# # ----------------
# fruits = ['apple', 'kiwi', 'orange']
# fruit1, fruit2, fruit3 = fruits
# print(fruit1, fruit2, fruit3)


# ----------------
# </> 22
# ----------------
# fruits = ('apple', 'kiwi', 'orange')
# fruits[0] = 'cherry'  # TypeError: 'tuple' object does not support item assignment
# fruits[1:] = 'cherry'  # TypeError: 'tuple' object does not support item assignment
# del fruits[0]  # TypeError: 'tuple' object doesn't support item deletion
# del fruits # OK

# fruits = ('apple', ['kiwi', 'orange'])
# fruits[1][0] = 'cherry'
# print(fruits)  # ('apple', ['cherry', 'orange']
# fruits[1] = [] # TypeError: 'tuple' object does not support item assignment

# ----------------
# </> 23
# ----------------

# fruit_genus = {
#     'malus': {
#         'scientific': {
#             'malus angustifolia': {
#                 'common_name': 'southern crabapple',
#                 'distribution': 'southern united states'
#             },
#             'malus brevipes': {
#                 'common_name': 'broadleaf crabapple',
#                 'distribution': 'eastern united states'
#             },
#         }
#     }
# }

# pprint(fruit_genus)


# ----------------
# </> 24
# ----------------

# d = {
#     "TurtleBot3": "mobile",
#     "UR10": "industrial",
#     "Gapter": "aerial",
#     "TurtleBot3": "aerial",
#     "greeting": "hello",
#     1: "integer",
#     2.5: "float",
#     int: "type",
#     True: True,
#     (1, 2, 3): (1, 2, 3),
#     [1, 2, 3]: "hello", # TypeError: unhashable type: 'list'
# }

# pprint(d)

# ----------------
# </> 25
# ----------------
# d = dict()
# pprint(d)  # {}
# d = dict(a=1, b="hello")
# pprint(d)  # {'a': 1, 'b': 'hello'}
# # ----------------
# # ----------------
# d = dict({"a": 4, "b": 5})
# pprint(d)  # {'a': 4, 'b': 5}
# d = dict({"a": 4, "b": 5}, c=3, d=4)
# pprint(d)  # {'a': 4, 'b': 5, 'c': 3, 'd': 4}
# # e = dict(d, c=3, d=4)
# # pprint(e)# {'a': 4, 'b': 5, 'c': 3, 'd': 4}
# # ----------------
# # ----------------
# d = dict([["x", 5], ("y", -5)])
# pprint(d)  # {'x': 5, 'y': -5}
# d = dict([("x", 5), ("y", -5)])
# pprint(d)  # {'x': 5, 'y': -5}
# d = dict([("x", 5), ("y", -5)], z=8)
# pprint(d)  # {'x': 5, 'y': -5, 'z': 8}

# ----------------
# </> 26
# ----------------
# d = {x: x**2 for x in [1, 2, 3]}
# pprint(d)  # {1: 1, 2: 4, 3: 9}
# d = {x.upper(): x * 2 for x in "oops"}
# pprint(d)  # {'O': 'oo', 'P': 'pp', 'S': 'ss'}

# ----------------
# </> 27
# ----------------
# fruit_genus = {}

# fruit_genus["watermelon"] = "citrus"
# fruit_genus["orange"] = "citrus"
# fruit_genus["watermelon"] = "citrullus"
# pprint(fruit_genus)
# # ----------------
# # ----------------
# fruit_genus_1 = {"watermelon": "citrullus"}
# fruit_genus_2 = {"lemon": "citrus"}     # dictionary
# fruit_genus_1.update(fruit_genus_2)
# pprint(fruit_genus_1)  # {'lemon': 'citrus', 'watermelon': 'citrullus'}

# ----------------
# </> 28
# ----------------
# fruit_genus = {
#     "watermelon": "citrullus",
#     "orange": "citrus",
#     "plum": "prunus",
#     "lemon": "citrus",
# }

# print(fruit_genus['orange'])
# print(fruit_genus['apple'])  # KeyError: 'apple'

# print(fruit_genus.get("orange"))  # citrus
# print(fruit_genus.get("apple"))  # None
# print(fruit_genus.get("apple", "apple is not a key"))  # apple is not a key

# ----------------
# </> 29
# ----------------
# fruit_genus = {
#     "watermelon": "citrullus",
#     "orange": "citrus",
#     "plum": "prunus",
#     "lemon": "citrus",
# }

# def print_not_found(key):
#     print(f"{key} is not found")

# fruit_genus.get('apple', print_not_found("apple")) # apple is not found

# ----------------
# </> 30
# ----------------
# fruit_genus = {
#     "orange": "citrus",
#     "plum": "prunus",
# }

# fruit_keys = fruit_genus.keys()
# print(fruit_keys)  # dict_keys(['orange', 'plum'])
# fruit_genus["lemon"] = "citrus"
# print(fruit_keys)  # dict_keys(['orange', 'plum', 'lemon'])

# ----------------
# </> 31
# ----------------
# fruit_genus = {
#     "orange": "citrus",
#     "plum": "prunus",
# }

# # iterate directly over the keys
# for key in fruit_genus:
#     print(key, end=" ")  # orange plum

# print()

# # view object
# keys = fruit_genus.keys()

# for key in keys:
#     print(key, end=" ")  # orange plum

# print()

# ----------------
# </> 32
# ----------------
# fruit_genus = {
#     "orange": "citrus",
#     "plum": "prunus",
# }

# # iterate directly over the keys
# for key in fruit_genus:
#     print(fruit_genus[key], end=" ")  # citrus prunus

# print()

# # view object
# values = fruit_genus.values()

# for value in values:
#     print(value, end=" ")  # citrus prunus

# print()

# ----------------
# </> 33
# ----------------
# fruit_genus = {
#     "orange": "citrus",
#     "plum": "prunus",
# }

# # iterate directly over the keys
# for key in fruit_genus:
#     print(key, fruit_genus[key], end=" ")  # orange citrus plum prunus

# print()

# # view object
# items = fruit_genus.items()

# for key, value in items:
#     print(key, value, end=" ")  # orange citrus plum prunus

# print()

# ----------------
# </> 34
# ----------------
# fruit_genus = {
#     "orange": "citrus",
#     "plum": "prunus",
# }

# print(fruit_genus.pop("orange"))  # citrus
# print(fruit_genus)  # {'plum': 'prunus'}
# print(fruit_genus.pop("apple", "apple not found"))  # apple not found
# print(fruit_genus.pop("apple"))  # KeyError: 'apple'

# ----------------
# </> 35
# ----------------
# fruit_genus = {
#     "plum": "prunus",
#     "orange": "citrus",
# }

# print(fruit_genus.popitem())  # ('orange', 'citrus')
# print(fruit_genus)  # {'plum': 'prunus'}
# fruit_genus["apple"] = "malus"
# print(fruit_genus)  # {'plum': 'prunus', 'apple': 'malus'}
# print(fruit_genus.popitem())  # ('apple', 'malus')
# print(fruit_genus)  # {'plum': 'prunus'}

# ----------------
# </> 36
# ----------------
# fruit_genus = {
#     "plum": "prunus",
#     "orange": "citrus",
#     "apple": "citrus",
#     "plantain": "musa",
# }

# ----------------
# </> 37
# ----------------
# fruits = {"watermelon", "orange", "orange"}
# print(fruits)

# ----------------
# </> 38
# ----------------
# fruits = {"watermelon", "orange"}  # {'watermelon', 'orange'}
# print(fruits)
# fruits = {("watermelon", "orange")}  # {('watermelon', 'orange')}
# print(fruits)

# fruits = {}
# print(type(fruits))  # <class 'dict'>

# ----------------
# </> 39
# ----------------
# s = set(["apple", "orange"])    # {'apple', 'orange'}
# print(s)
# s = set()                       # set()
# print(s)
# s = set(range(0, 10, 2))        # {0, 2, 4, 6, 8}
# print(s)
# s = set((1, 4, 9, 16, 4))       # {1, 4, 9, 16}
# print(s)
# s = set("apple")                # {'a', 'e', 'l', 'p'}
# print(s)

# ----------------
# </> 40
# ----------------
# fruits = {"apple", "orange", "apple"}

# for item in fruits:
#     print(item, end=" ")  # apple orange

# print()

# ----------------
# </> 41
# ----------------
# fruits = {"apple", "orange", "apple"}
# fruits.add("kiwi")

# # # Be careful
# # fruits = {}
# # fruits.add('kiwi') # AttributeError: 'dict' object has no attribute 'add'

# ----------------
# </> 42
# ----------------
# fruits = {"apple", "orange", "apple"}
# fruits.remove("apple")
# print(fruits)  # {'orange'}
# fruits.remove("banana")  # KeyError: 'banana'
# # ----------------
# # ----------------
# fruits = {"apple", "orange", "apple"}
# fruits.discard('banana')
# # ----------------
# # ----------------
# fruits = {"apple", "orange"}
# fruits.pop()
# print(fruits)  # 'apple' or 'orange'
# # ----------------
# # ----------------
# fruits = {"apple", "orange"}
# fruits.clear()
# print(fruits) # set()

# ----------------
# </> 43
# ----------------
# s1 = {0, 2, 4, 6, 8}
# s2 = {4, 6, 8, 10}
# l = [4, 6, 8, 10]

# print(s1.union(s2)) # {0, 2, 4, 6, 8, 10}
# print(s1 | s2)      # {0, 2, 4, 6, 8, 10}
# print(s1.union(l))  # {0, 2, 4, 6, 8, 10}
# # print(s1 | l)       # TypeError: unsupported operand type(s) for |: 'set' and 'list'

# ----------------
# </> 44
# ----------------
# s1 = {0, 2, 4, 6, 8}
# s2 = {4, 6, 8, 10}
# l = [0, 2, 4, 6, 8, 10]

# s1.update(s2)   # {0, 2, 4, 6, 8, 10}
# s1 |= s2        # {0, 2, 4, 6, 8, 10}
# s1.update(l)    # {0, 2, 4, 6, 8, 10}
# print(s1)       # {0, 2, 4, 6, 8, 10}
# # s1 |= l         # TypeError: unsupported operand type(s) for |=: 'set' and 'list'

# ----------------
# </> 45
# ----------------