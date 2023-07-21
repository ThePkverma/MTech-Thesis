
file_path = "RawMq.txt"

# Open the file in read mode
with open(file_path, "r") as file:
    # Read the contents of the file as a string
    mqstr = file.read()

file_path = "RawCq.txt"

# Open the file in read mode
with open(file_path, "r") as file:
    # Read the contents of the file as a string
    cqstr = file.read()

file_path = "RawGq.txt"

# Open the file in read mode
with open(file_path, "r") as file:
    # Read the contents of the file as a string
    gqstr = file.read()

file_path = "RawHq.txt"

# Open the file in read mode
with open(file_path, "r") as file:
    # Read the contents of the file as a string
    hqstr = file.read()



values = [
        ['Matrix', 'np.array'],
        ['(t)',''],

         ['q3(q4)','q3'],
         ['q2(q4)','q2'],

         ['Derivative(q3, q4)','u34'],
         ['Derivative(q2, q4)','u24'],

         ['Derivative(q3, (q4, 2))', 'a34'],
         ['Derivative(q2, (q4, 2))', 'a34'],

         ['Derivative(x, t)','ux'],
         ['Derivative(phi, t)','up'],
         ['Derivative(z, t)','uz'],
         ['Derivative(q1, t)','u1'],
         ['Derivative(q4, t)','u4'],
         ['Derivative(qa1, t)','ua1'],
         ['Derivative(qa2, t)','ua2'],
         
         ['Derivative(x, (t, 2))','ax'],
         ['Derivative(phi, (t, 2))','ap'],
         ['Derivative(z, (t, 2))','az'],
         ['Derivative(q1, (t, 2))','a1'],
         ['Derivative(q4, (t, 2))','a4'],
         ['Derivative(qa1, (t, 2))','aa1'],
         ['Derivative(qa2, (t, 2))','aa2'],

         ['mw', 'self.mw'], 
         ['m1', 'self.m1'], 
         ['m2', 'self.m2'], 
         ['m3', 'self.m3'], 
         ['mb', 'self.mb'], 
         ['ma1', 'self.ma1'], 
         ['ma2', 'self.ma2'], 
         ['l01', 'self.l01'], 
         ['l12', 'self.l12'], 
         ['l24', 'self.l24'], 
         ['l13', 'self.l13'], 
         ['l34', 'self.l34'], 
         ['la1', 'self.la1'], 
         ['la2', 'self.la2'],  
         ['g', 'self.g'], 
         ['d', 'self.d'], 
         ['beta', 'self.beta'], 
         ['M_c', 'self.M_c'], 
         ['kt', 'self.kt'], 
          ]

for substitution in values:
    mqstr = mqstr.replace(substitution[0], substitution[1])
    cqstr = cqstr.replace(substitution[0], substitution[1])
    gqstr = gqstr.replace(substitution[0], substitution[1])
    hqstr = hqstr.replace(substitution[0], substitution[1])

file_path = 'Mq.txt'
with open(file_path, "w") as file:
    # Write the string to the file
    file.write(mqstr)
file_path = 'Cq.txt'
with open(file_path, "w") as file:
    # Write the string to the file
    file.write(cqstr)
file_path = 'Gq.txt'
with open(file_path, "w") as file:
    # Write the string to the file
    file.write(gqstr)
file_path = 'Hq.txt'
with open(file_path, "w") as file:
    # Write the string to the file
    file.write(hqstr)

# print(cqstr)