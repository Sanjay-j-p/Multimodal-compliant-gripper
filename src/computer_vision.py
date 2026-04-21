import numpy as np
import open3d as o3d
import cv2
import pyrealsense2 as rs
from inference_sdk import InferenceHTTPClient
import base64
import time
import argparse



import paho.mqtt.client as mqtt
import time
 

 
ROBOFLOW_API_KEY = "X4VNgKBmSlWpGrFn9OAe"
WORKSPACE_NAME = "yalamanchilivarshitha-gmail-com"
WORKFLOW_ID = "general-segmentation-api"
 
 

 
class RoboflowGraspAnalyzer:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        
        # Get RealSense Intrinsics for projection
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
        
        self.pc = rs.pointcloud()
 
    def get_roboflow_mask(self, image_path, target_classes):
        
        print(target_classes)
        print(type(target_classes))
        client = InferenceHTTPClient(
            api_url="https://serverless.roboflow.com",
            api_key=ROBOFLOW_API_KEY
)
        result = client.run_workflow(
            workspace_name=WORKSPACE_NAME,
            workflow_id=WORKFLOW_ID,
            images={"image": image_path},
            parameters={"classes": list(target_classes)},
            use_cache=False
        )
       
        data = result[0] if isinstance(result, list) else result
        
       
        if 'predictions' in data and 'image' in data['predictions']:
            w = data['predictions']['image']['width']
            h = data['predictions']['image']['height']
            print(f"Canvas initialized with dimensions: {w}x{h}")
        if not w or not h:  
            h, w = 480, 640
            print("Warning: using default dimensions 640x480")
                #
        temp_mask = np.zeros((h, w), dtype=np.uint8)
        if 'annotated_image' in data:
            base64_str = data['annotated_image']
            if "," in base64_str:
                base64_str = base64_str.split(",")[1]
        
            img_bytes = base64.b64decode(base64_str)
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            annotated_img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        
            if annotated_img is not None:
            
                cv2.imshow("1. Annotated Detection (Visual)", annotated_img)
        else:
            print("Could not find the annotated_image string.")
        
        preds_list = data.get('predictions', {}).get('predictions', [])
        if not preds_list and isinstance(data.get('predictions'), list):
            preds_list = data['predictions']
        
        found_any = False
        for pred in preds_list:
           
            if isinstance(pred, dict) and 'points' in pred:
                points = pred['points']
            
               
                pts = np.array([[int(p['x']), int(p['y'])] for p in points], dtype=np.int32)
            
                
                cv2.fillPoly(temp_mask, [pts], 255)
                found_any = True
                print(f"Object '{pred.get('class')}' drawn on matrix.")
        
 
        if found_any:
            
            cv2.imshow("Binary Matrix (Resized for View)", temp_mask)
        
           
            cv2.imwrite("robotics_occupancy_grid.png", temp_mask)
            print("Full resolution matrix saved as 'robotics_occupancy_grid.png'")
            return temp_mask
        else:
            print("No points were found to draw.")
        
        return None
 
    def analyze_grasp(self, color_frame, depth_frame, mask,depth_normalized):
        masked_depth = depth_normalized[mask > 0]  
        masked_depth = masked_depth[masked_depth > 0]  
        print(f"mask shape: {mask.shape}")
        print(f"depth_normalized shape: {depth_normalized.shape}")
        if len(masked_depth) == 0:
            return None, "No valid depth under mask",0
        self.pc.map_to(color_frame)
        points = self.pc.calculate(depth_frame)
        v = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        mask_flat = mask.ravel() > 0
        target_v = v[mask_flat]
        target_v = target_v[target_v[:, 2] > 0]
        
        if len(target_v) < 150:
            return None, "Point cloud too sparse",0
 
        top_idx = np.argmin(target_v[:, 2])
        top_point_meters = target_v[top_idx]
        wz = (top_point_meters[2] + 0.35)*1000
        center_3d = np.mean(target_v, axis=0)
 
       
        fx, fy = self.intrinsic.get_focal_length()
        cx, cy = self.intrinsic.get_principal_point()
        u = int((center_3d[0] * fx / center_3d[2]) + cx)
        v = int((center_3d[1] * fy / center_3d[2]) + cy)
 
 
 
        z_std = np.std(masked_depth)
        z_mean = np.mean(masked_depth)
        print(f"Depth mean: {z_mean:.1f}  std: {z_std:.1f}")
 
        near = masked_depth[masked_depth < z_mean]
        far  = masked_depth[masked_depth >= z_mean]
 
        if len(near) > 0 and len(far) > 0:
            gap = np.mean(far) - np.mean(near)
            print(f"Depth gap: {gap:.1f}")
            if gap > 7:  
                print("hollow detected")
                return center_3d, "FINGER_GRASP",wz
 
        if z_std > 30:  
            return center_3d, "FINGER_GRASP",wz
 
        
        
        
 
        return center_3d, "VACCUM",wz
 
    def run(self, classes_to_find):
        print(f"Live. Classes: {classes_to_find}. Press 'C' to trigger Roboflow + Grasp Analysis.")
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                color_f = aligned.get_color_frame()
                depth_f = aligned.get_depth_frame()
                if not color_f or not depth_f: continue
 
                color_img = np.asanyarray(color_f.get_data())
                cv2.imshow("RealSense Live Feed", color_img)
                depth_image = np.asanyarray(depth_f.get_data())
                depth_clipped = np.where(depth_image == 65535, 0, depth_image)
 
                
                depth_clipped = np.clip(depth_clipped, 150, 800)
 
               
                depth_normalized = ((depth_clipped - 150) / (800 - 150) * 255).astype(np.uint8)
 
                
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('c'):  
                    save_path = "captured_frame.png"  
                    cv2.imwrite(save_path, color_img)
                    print(f"Picture saved as {save_path}")
                    time.sleep(0.5)
                    mask = self.get_roboflow_mask(save_path, classes_to_find)
                    
                    if mask is not None:
                        
                        target_3d, g_type ,z_val= self.analyze_grasp(color_f, depth_f, mask,depth_normalized)
                        
                        
                        res_img = color_img.copy()
                       
                        res_img[mask > 0] = cv2.addWeighted(res_img[mask > 0], 0.5,
                                                           np.full_like(res_img[mask > 0], (0,255,0)), 0.5, 0)
                        if g_type == "FINGER_GRASP":
                            fx, fy = self.intrinsic.get_focal_length()
                            cx, cy = self.intrinsic.get_principal_point()
                            u = int((target_3d[0] * fx / target_3d[2]) + cx)
                            v = int((target_3d[1] * fy / target_3d[2]) + cy)
                            cv2.drawMarker(res_img, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
                            cv2.putText(res_img, f"GRASP: {g_type}", (20, 50),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            cv2.putText(res_img, f"point: {(u, v, int(z_val))}", (20, 80),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        if g_type == "VACCUM":
                            fx, fy = self.intrinsic.get_focal_length()
                            cx, cy = self.intrinsic.get_principal_point()
                            u = int((target_3d[0] * fx / target_3d[2]) + cx)
                            v = int((target_3d[1] * fy / target_3d[2]) + cy)
                            
                            cv2.drawMarker(res_img, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
                            cv2.putText(res_img, f"GRASP: {g_type}", (20, 50),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            cv2.putText(res_img, f"point: {(u, v, int(z_val))}", (20, 80),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        
                        cv2.imshow("Workflow Result", res_img)
                        cv2.waitKey(0)
                        cv2.destroyWindow("Workflow Result")
                    else:
                        print("Roboflow found no targets.")
 
                elif key == ord('q'):
                    break
        finally:
 
            self.pipeline.stop()
            cv2.destroyAllWindows()
 
if __name__ == "__main__":
    
    analyzer = RoboflowGraspAnalyzer()
    analyzer.run(["phone","mouse", "Tape","Box","Plastic"])