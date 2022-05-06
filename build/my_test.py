# my_test.py
 
class Person:
    def __init__(self):
        self.name = 'wdx'
        self.age = 23
 
    def set_msg(self, name, age):
        self.name = name
        self.age = age
 
    def get_msg(self):
        print('python: name={}, age={}'.format(self.name, self.age))
        return self.name, self.age
