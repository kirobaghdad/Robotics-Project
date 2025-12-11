import cv2

MIN_MATCH_COUNT = 3

class VisualSearch:
    def __init__(self, object_img_path):
        self.orb = cv2.ORB_create()
        
        # read the image
        img = cv2.imread(object_img_path)
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        self.img_path = object_img_path  # Store path for debugging
        self.target_keypoints, self.target_desriptors = self.orb.detectAndCompute(gray_img, None)

        
    def search_object(self, camera_frame):
        
        # Use the camera frame directly instead of reading from file
        # convert to gray scale
        gray_frame = cv2.cvtColor(camera_frame, cv2.COLOR_BGR2GRAY)
        
        # We choose the ORB detector 
        # beacuse it is fast 
        # good enough accuracy
        
        
        # finding interesting points in the camera frame
        # keypoints -> where interesting points are located
        # desriptors -> how interesting points look like
        keypoints, desriptors = self.orb.detectAndCompute(gray_frame, None)
        
        # using a brute force matcher
        # NORM_HAMMING is ditance calculation method for ORB
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)
        matches = bf.match(self.target_desriptors, desriptors)
        
        # sorting the matches
        matches = sorted(matches, key = lambda x:x.distance)
        
        # getting best 10 matches
        best_matches = matches[:10]
        
        # Optional: draw matches for debugging
        if len(matches) > 0:
            target_img = cv2.imread(self.img_path) if hasattr(self, 'img_path') else None
            if target_img is not None:
                result_img = cv2.drawMatches(target_img, self.target_keypoints, camera_frame, keypoints, matches[:10], None, flags = 2)
                cv2.imshow("Visual Search Result", result_img)
                cv2.waitKey(1)
        
        # decision with debug info
        print(f"Found {len(matches)} total matches, {len(best_matches)} best matches")
        if len(best_matches) > MIN_MATCH_COUNT:
            return True
        else:
            return False