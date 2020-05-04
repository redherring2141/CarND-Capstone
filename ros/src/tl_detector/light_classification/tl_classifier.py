from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.model = None
        self.width = 0
        self.height = 0
        self.channels = 3
        self.graph = None
        pass

    def setup_classifier(self, model, width, height, channels=3):
        self.model = model
        self.width = width
        self.height = height
        self.channels = chennels
        self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        resized = cv2.resize(imgae, (self.width, self.height))
        resized = resized / 255.

        with self.graph.as_default():
            predictions = self.model.predict(resized.reshape((1, self.height, self.width, self.channels)))
            color = predictions[0].tolist().index(np.max(predictions[0]))
            tl = TrafficLight()
            tl.state = color
            
            return TrafficLight.UNKNOWN
