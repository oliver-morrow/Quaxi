import glob
import os
import pickle
import xml.etree.ElementTree as ET
from os import listdir, getcwd
from os.path import join

dirs = ['train', 'val']
classes = ['duck_regular', 'sign_noentry', 'sign_oneway_left', 'sign_oneway_right', 'sign_stop', 'sign_yield']

def getImagesInDir(dir_path):
    image_list = []
    for filename in glob.glob(dir_path + '/*.jpg'):
        image_list.append(filename)

    return image_list

def convert(size, box):
    dw = 1./(size[0])
    dh = 1./(size[1])
    x = (box[0] + box[1])/2.0 - 1
    y = (box[2] + box[3])/2.0 - 1
    w = box[1] - box[0]
    h = box[3] - box[2]
    x = x*dw
    w = w*dw
    y = y*dh
    h = h*dh
    return (x,y,w,h)

def convert_annotation(dir_path, output_path, image_path):
    basename = os.path.basename(image_path)
    basename_no_ext = os.path.splitext(basename)[0]

    in_file = open(dir_path + '/' + basename_no_ext + '.xml')
    out_file = open(output_path + basename_no_ext + '.txt', 'w')
    tree = ET.parse(in_file)
    root = tree.getroot()
    size = root.find('size')
    w = int(size.find('width').text)
    h = int(size.find('height').text)

    for obj in root.iter('object'):
        difficult = obj.find('difficult').text
        cls = obj.find('name').text
        if cls not in classes or int(difficult)==1:
            continue
        cls_id = classes.index(cls)
        xmlbox = obj.find('bndbox')
        b = (float(xmlbox.find('xmin').text), float(xmlbox.find('xmax').text), float(xmlbox.find('ymin').text), float(xmlbox.find('ymax').text))
        bb = convert((w,h), b)
        out_file.write(str(cls_id) + " " + " ".join([str(a) for a in bb]) + '\n')
    
    in_file.close()
    out_file.close()

def process_dataset():
    """Process the entire dataset with progress reporting"""
    # go to location of dataset
    cwd = getcwd()
    print(f"Working directory: {cwd}")

    for dir_path in dirs:
        full_dir_path = cwd + '/' + dir_path
        output_path = full_dir_path +'/yolo/'
        
        print(f"Processing directory: {dir_path}")
        print(f"Looking for images in: {full_dir_path}")

        if not os.path.exists(output_path):
            os.makedirs(output_path)
            print(f"Created output directory: {output_path}")

        image_paths = getImagesInDir(full_dir_path)
        print(f"Found {len(image_paths)} images")
        
        if len(image_paths) == 0:
            print(f"Warning: No images found in {full_dir_path}")
            continue
            
        list_file = open(full_dir_path + '.txt', 'w')

        for i, image_path in enumerate(image_paths):
            if i % 100 == 0:
                print(f"Processing image {i+1}/{len(image_paths)}")
                
            list_file.write(image_path + '\n')
            try:
                convert_annotation(full_dir_path, output_path, image_path)
            except Exception as e:
                print(f"Error processing {image_path}: {e}")
                
        list_file.close()
        print(f"Finished processing: {dir_path}")

if __name__ == "__main__":
    process_dataset()
