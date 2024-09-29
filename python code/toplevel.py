class TopLevelSystem:
    def __init__(self, motionPlan):
        self.motionPlan = motionPlan
        self.sequenceCounter = 1
        self.terminate = False

    def get_sequence_pair(self):
        if self.sequenceCounter + 1 > len(self.motionPlan):
            self.terminate = True
            return None
        else: 
            sequencePair = [self.motionPlan[self.sequenceCounter - 1], self.motionPlan[self.sequenceCounter]]
            self.sequenceCounter += 1
            return sequencePair
        