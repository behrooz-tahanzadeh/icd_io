import rospy
from std_srvs.srv._Trigger import Trigger, TriggerResponse
import rosgraph




class TopicListService:
	
	def __init__(self, fieldSep, recordSep):
		self.master = rosgraph.Master('/itech_topic_list')
		
		self.fieldSep = fieldSep
		self.recordSep = recordSep
	#eof
	
	
	
	def run(self):
		return rospy.Service('itech_topic_list', Trigger, self.callback)
	#eof
	
	
	
	def callback(self, req):
		rospy.loginfo("New request...")
		outputStr = self.format(self.master.getPublishedTopics(''))
		return TriggerResponse(True, outputStr)
	#eof
	
	
	
	def format(self, topics):
		content = []
		
		for row in topics:
			content.append(row[0]+self.fieldSep+row[1])
		
		return self.recordSep.join(content)
	#eof
	
#eoc