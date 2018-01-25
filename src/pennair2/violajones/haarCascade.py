#https://arxiv.org/pdf/1611.07791.pdf
#https://github.com/Simon-Hohberg/Viola-Jones/tree/master/violajones

#import violajones.something

def enum(**enums):
    return type('Enum', (), enums)

# todo change these to fit our needs
FeatureType = enum(TWO_VERTICAL=(1, 2), TWO_HORIZONTAL=(2, 1), THREE_HORIZONTAL=(3, 1), THREE_VERTICAL=(1, 3), FOUR=(2, 2))
FeatureTypes = [FeatureType.TWO_VERTICAL, FeatureType.TWO_HORIZONTAL, FeatureType.THREE_VERTICAL, FeatureType.THREE_HORIZONTAL, FeatureType.FOUR]

class HaarLikeFeature:

	# todo determine our params
	def __init__(self, feature_rotation, position, radius, threshold, polarity):
		pass

	# todo determine our params and how to calculate score
	def getScore(self, int_img):
		pass

	# todo determine our params and how to get vote
	def getVote(self, int_img):
		pass
