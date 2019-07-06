// def plot_one_box_on_image(self, x, img, color=None, label=None, line_thickness=None, labelclass=None):
//         # Plots one bounding box on image img
//         tl = line_thickness or round(0.002 * max(img.shape[0:2])) + 1  # line thickness
//         color = color or [random.randint(0, 255) for _ in range(3)]
//         c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
//         cv2.rectangle(img, c1, c2, color, thickness=int(tl) )
//         if label:
//             tf = max(tl - 1, 1)  # font thickness
//             t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=int(tf) )[0]
//             c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
//             cv2.rectangle(img, c1, c2, color, -1)  # filled
//             cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=int(tf), lineType=cv2.LINE_AA)

//             ### calculate real world position relative to self vehicle 
//             # pos = cal_real_pos(int(x[0] + x[2])/2, int(x[3]), img.shape)

//             # u = int(x[0] + x[2])/2
//             # v = int(x[3])
//             # u = np.array([u])
//             # v = np.array([v])
//             # lo, la = self.cam.img2world(u, v)
//             # pos = (lo[0], la[0], labelclass)
//             pos = self.calc_real_world_pos(x, labelclass)
//             lo = pos[0]
//             la = pos[1]
            
//             # cv2.putText(img, 'lo %.1f la %.1f'%(pos[1], pos[0]), ((x[0] + x[2])/2, x[3]+2), 0, tl/3, [255, 0, 0], thickness=tf, lineType=cv2.LINE_AA)
//             cv2.putText(img, 'lo %.1f'%(lo), ((x[0] + x[2])/2, x[3]+5), 0, tl/3 - 0.2, [255, 0, 0], thickness=int(tf), lineType=cv2.LINE_AA)
//             cv2.putText(img, 'la %.1f'%(la), ((x[0] + x[2])/2, x[3]+20), 0, tl/3 - 0.2, [255, 0, 0], thickness=int(tf), lineType=cv2.LINE_AA)
//             return pos
//         else:
//             return None


//         TODO: translate to Detection2D
//         self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.classes))]