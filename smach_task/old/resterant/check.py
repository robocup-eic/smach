with open("main.py") as fp:
    for i, line in enumerate(fp):
        if "\xe2" in line:
            print( i, repr(line))

            
            
            
            
        # print(f"Ok, I’m getting you a {res_listen['object']}") 372