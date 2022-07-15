# a = ["water","water", "coke", "coke", "coke", "sprite"]
a = ['milk', 'milk', 'milk']

b = list(set(a))
b.sort(reverse= True, key=a.count)
print(a)
print(b)