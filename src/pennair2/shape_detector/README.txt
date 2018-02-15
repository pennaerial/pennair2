Running the program:
Mac: python classify.py <image file> <parameter> <ignores>
Not Mac: ???

Parameter:
- Input any one of the colors to detect all shapes of that color (note: green and brown not implemented) or input "all" to detect all shapes of all colors.

Ignores:
- Only checked if parameter="all", it's all the colors you don't want to look for to handle the fact that the background isn't always green

Known issues:
- Cannot detect shapes with same color as background

Other:
- Some test pictures are provided
- Program likes images with 300 ppi, otherwise might not work

To do:
- Figure out how to detect shapes with same color as background
- Implement more colors
- Figure out how HSV works so can make changes at the competition if needed
- Allow setting background color to avoid whitewashing the whole output regardless of surface you're flying over
- Label shapes with their colors