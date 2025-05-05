# -----------------
# Inheritance
# -----------------
class Animal:
    def __init__(self, can_fly=False, can_swim=False):
        self._can_fly = can_fly
        self._can_swim = can_swim

    def eat(self):
        print("Animal is eating")

class Dog(Animal):
    def __init__(self, name, can_fly=False, can_swim=False):
        super().__init__(can_fly, can_swim)
        self._name = name

    def bark(self):
        print(f"{self._name} is barking")

if __name__ == "__main__":
    bernie = Dog("Bernie", False, True)   
    print()

# -----------------
# ABC
# -----------------

# from abc import ABC, abstractmethod

# class Animal(ABC):  # Abstract Base Class
#     @abstractmethod
#     def make_sound(self):
#         """Abstract method that must be implemented in subclasses"""
#         pass  # No implementation in the base class

# class Dog(Animal):
#     def make_sound(self):
#         return "Woof! Woof!"

# class Cat(Animal):
#     def Make_sound(self):
#         return "Meow!"

# if __name__ == "__main__":
#     # animal = Animal()  # TypeError: Can't instantiate abstract class
    
#     dog = Dog()
#     print(dog.make_sound())  # Woof! Woof!
    
#     cat = Cat() # TypeError: Can't instantiate abstract class Cat
#     print(cat.Make_sound()) 