import cv2
import numpy as np
import os

MIN_MATCH_COUNT = 10
MIN_HOMOGRAPHY_MATCHES = 10

class VisualSearch:
    def __init__(self, object_img_paths):
        # Use ORB for fast feature detection
        self.detector = cv2.ORB_create(
            nfeatures=2000,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=15,
            patchSize=31
        )
                
        # # Support multiple reference images
        # if isinstance(object_img_paths, str):
        #     object_img_paths = [object_img_paths]
        
        object_img = os.listdir(object_img_paths)

        
        self.reference_data = []
        
        for img_path in object_img:
            img_path = os.path.join(object_img_paths, img_path)
            if os.path.exists(img_path):
                img = cv2.imread(img_path)
                if img is not None:
                    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    kp, desc = self.detector.detectAndCompute(gray_img, None)
                    
                    if desc is not None and len(desc) > 0:
                        self.reference_data.append({
                            'path': img_path,
                            'keypoints': kp,
                            'descriptors': desc,
                            'image': img
                        })
        
        if not self.reference_data:
            raise ValueError("No valid reference images found")


        
    def search_object(self, camera_frame):
        gray_frame = cv2.cvtColor(camera_frame, cv2.COLOR_BGR2GRAY)
        
        # Extract features from camera frame
        frame_kp, frame_desc = self.detector.detectAndCompute(gray_frame, None)
        
        if frame_desc is None or len(frame_desc) == 0:
            print("No features detected in camera frame")
            return False, None
        
        # Try matching against all reference images
        best_match_count = 0
        best_matches = []
        best_ref_data = None
        
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        for ref_data in self.reference_data:
            matches = bf.match(ref_data['descriptors'], frame_desc)
            
            if len(matches) > 0:
                # Filter matches by distance and apply ratio test
                if len(matches) >= 2:
                    # Lowe's ratio test to filter out ambiguous matches (good for walls)
                    good_matches = []
                    for i, m in enumerate(matches[:-1]):
                        if m.distance < 0.75 * matches[i+1].distance:  # Stricter ratio for walls
                            good_matches.append(m)
                else:
                    good_matches = [m for m in matches if m.distance < 50]
                
                good_matches = sorted(good_matches, key=lambda x: x.distance)[:30]
                
                # Geometric validation with homography
                if len(good_matches) >= MIN_HOMOGRAPHY_MATCHES:
                    src_pts = np.float32([ref_data['keypoints'][m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                    
                    dst_pts = np.float32([frame_kp[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                    
                    try:
                        # Stricter homography for wall rejection
                        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 2.0, 
                                                    maxIters=3000, confidence=0.995)
                        
                        if M is not None:
                            inliers = np.sum(mask)
                            if inliers > best_match_count:
                                best_match_count = inliers
                                best_matches = [good_matches[i] for i in range(len(good_matches)) if mask[i]]
                                best_ref_data = ref_data
                    except:
                        pass
        
        # Create visualization image
        result_img = None
        if best_ref_data and len(best_matches) > 0:
            ref_img = best_ref_data['image']
            h1, w1 = ref_img.shape[:2]
            h2, w2 = camera_frame.shape[:2]
            target_h, target_w = min(h1, h2), min(w1, w2)
            ref_resized = cv2.resize(ref_img, (target_w, target_h))
            cam_resized = cv2.resize(camera_frame, (target_w, target_h))
            result_img = cv2.drawMatches(ref_resized, best_ref_data['keypoints'], 
                                       cam_resized, frame_kp, best_matches[:20], None, flags=2)
        
        print(f"Best match: {best_match_count} geometrically verified matches")
        
        # Additional validation: check if matches are clustered (object-like)
        if best_ref_data and len(best_matches) > 0:
            match_points = [frame_kp[m.trainIdx].pt for m in best_matches]
            if len(match_points) > 4:
                # Calculate bounding box of matches
                x_coords = [p[0] for p in match_points]
                y_coords = [p[1] for p in match_points]
                bbox_area = (max(x_coords) - min(x_coords)) * (max(y_coords) - min(y_coords))
                frame_area = camera_frame.shape[0] * camera_frame.shape[1]
                
                # If matches cover too much area, likely wall matches
                if bbox_area > 0.8 * frame_area:
                    print("Matches too spread out - likely wall features")
                    best_match_count = 0
        
        # Fallback to template matching if few keypoints
        if best_match_count < MIN_MATCH_COUNT:
            found, template_img = self._template_matching_fallback(camera_frame)
            return found, template_img if template_img is not None else result_img
        
        return best_match_count >= MIN_MATCH_COUNT, result_img
    
    def _template_matching_fallback(self, camera_frame):
        """Template matching for objects with few features"""
        gray_frame = cv2.cvtColor(camera_frame, cv2.COLOR_BGR2GRAY)
        
        for ref_data in self.reference_data:
            ref_gray = cv2.cvtColor(ref_data['image'], cv2.COLOR_BGR2GRAY)
            
            # Multi-scale template matching
            for scale in [0.5, 0.75, 1.0, 1.25, 1.5]:
                h, w = ref_gray.shape
                resized = cv2.resize(ref_gray, (int(w * scale), int(h * scale)))
                
                if resized.shape[0] > gray_frame.shape[0] or resized.shape[1] > gray_frame.shape[1]:
                    continue
                    
                result = cv2.matchTemplate(gray_frame, resized, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(result)
                
                if max_val > 0.7:  # High confidence threshold
                    print(f"Template match confidence: {max_val:.3f}")
                    
                    # Create visualization
                    vis_img = camera_frame.copy()
                    top_left = max_loc
                    bottom_right = (top_left[0] + resized.shape[1], top_left[1] + resized.shape[0])
                    cv2.rectangle(vis_img, top_left, bottom_right, (0, 255, 0), 2)
                    
                    return True, vis_img
        
        return False, None