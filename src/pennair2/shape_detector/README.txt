Running the program:
Mac: python classify.py <file> <parameter>
Not Mac: ???

Parameter:
- Input any one of the colors to detect all shapes of that color (note: some colors have not been implemented) or input "all" to detect all shapes of all colors (that have been implemented). 

Known issues:
- When the image has too much noise, it will label noise as shapes. Ignore those and keep pressing keys to move on and it'll eventually get to the actual shapes
- Circles currently won't work, it always somehow finds edges in circles
- Cannot detect shapes with same color as background

To do:
- Fix the noise
- Figure out how to detect shapes with same color as background
- Fix circles problem