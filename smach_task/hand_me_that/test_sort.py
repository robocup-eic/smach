a = ["water","water", "coke", "coke", "coke", "sprite"]
b = list(set(a))
b.sort(key=a.count)
print(a)
print(b)