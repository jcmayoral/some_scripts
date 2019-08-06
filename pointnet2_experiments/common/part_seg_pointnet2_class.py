#!/usr/bin/python
'''
    Evaluate classification performance with optional voting.
'''
import tensorflow as tf
import numpy as np
import argparse
import socket
import importlib
import time
import os
import scipy.misc
import sys
import heapq
BASE_DIR = "/home/jose/experiments_ws/pointnet2"
ROOT_DIR = BASE_DIR
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))
import provider
#import modelnet_dataset
#import modelnet_h5_dataset

class ROSPartSegPointNet2:
    def __init__(self, num_point=1):
        #TODO ROS PARAMS
        parser = argparse.ArgumentParser()
        parser.add_argument('--gpu', type=int, default=0, help='GPU to use [default: GPU 0]')
        #parser.add_argument('--model', default='pointnet2_cls_ssg', help='Model name. [default: pointnet2_cls_ssg]')
        parser.add_argument('--batch_size', type=int, default=1, help='Batch Size during training [default: 16]')
        #parser.add_argument('--num_point', type=int, default=150, help='Point Number [256/512/1024/2048] [default: 1024]')
        model = "pointnet2_part_seg"
        parser.add_argument('--model_path', default='part_log/model.ckpt', help='model checkpoint file path [default: part_log/model.ckpt]')
        parser.add_argument('--dump_dir', default='dump', help='dump folder path [dump]')
        parser.add_argument('--normal', action='store_true', help='Whether to use normal information')
        parser.add_argument('--num_votes', type=int, default=12, help='Aggregate classification scores from multiple rotations [default: 1]')
        FLAGS = parser.parse_args()
        self.is_initialize = False

        self.is_initialize = False
        self.BATCH_SIZE = FLAGS.batch_size
        self.NUM_POINT = num_point
        self.MODEL_PATH = FLAGS.model_path
        self.GPU_INDEX = FLAGS.gpu
        self.NUM_VOTES = FLAGS.num_votes
        self.MODEL = importlib.import_module(model) # import network module
        self.DUMP_DIR = FLAGS.dump_dir
        self.NORMAL = FLAGS.normal

        if not os.path.exists(self.DUMP_DIR): os.mkdir(self.DUMP_DIR)
        #os.system('cp %s %s' % (MODEL_FILE, LOG_DIR)) # bkp of model def
        #os.system('cp train.py %s' % (LOG_DIR)) # bkp of train procedure
        self.LOG_FOUT = open(os.path.join(self.DUMP_DIR, 'part_log_evaluate.txt'), 'w')
        self.LOG_FOUT.write(str(FLAGS)+'\n')
        self.NUM_CLASSES = 50



        self.seg_classes = ['Earphone','Motorbike','Rocket','Car','Laptop','Cap','Skateboard','Mug','Guitar','Bag','Lamp','Table','Airplane','Pistol','Chair', 'Knife']


        #self.SHAPE_NAMES = [line.rstrip() for line in \
        #open(os.path.join(ROOT_DIR, 'data/modelnet40_ply_hdf5_2048/shape_names.txt'))]
        #HOSTNAME = socket.gethostname()
        # Shapenet official train/test split
        #print "PATH ", os.path.join(BASE_DIR, 'data/modelnet40_ply_hdf5_2048/train_files.txt')
        #self.DATA_PATH = os.path.join(ROOT_DIR, 'data', 'shapenetcore_partanno_segmentation_benchmark_v0_normal')
        #self.TEST_DATASET = part_dataset_all_normal.PartNormalDataset(root=DATA_PATH, npoints=NUM_POINT, classification=False, split='test')



    def log_string(self,out_str):
        self.LOG_FOUT.write(out_str+'\n')
        self.LOG_FOUT.flush()
        #print(out_str)

    def stop_call(self):
        self.LOG_FOUT.close()

    def evaluate(self, data):
        if not self.is_initialize:
            with tf.device('/gpu:'+str(self.GPU_INDEX)):
                pointclouds_pl, labels_pl = self.MODEL.placeholder_inputs(self.BATCH_SIZE, self.NUM_POINT)
                is_training_pl = tf.placeholder(tf.bool, shape=())

                # simple model
                pred, end_points = self.MODEL.get_model(pointclouds_pl, is_training_pl)
                loss=self.MODEL.get_loss(pred, labels_pl)
                self.saver = tf.train.Saver()

            # Create a session
            self.config = tf.ConfigProto()
            self.config.gpu_options.allow_growth = True
            self.config.allow_soft_placement = True
            self.sess = tf.Session(config=self.config)

            # Restore variables from disk.
            self.saver.restore(self.sess, self.MODEL_PATH)
            self.log_string("Model restored.")

            self.ops = {'pointclouds_pl': pointclouds_pl,
               'labels_pl': labels_pl,
               'is_training_pl': is_training_pl,
               'pred': pred,
               'loss': loss}

            self.is_initialize = True

        self.eval_one_time(1, data)

    def eval_one_time(self, topk=1, data=0):
        is_training = False

        # Make sure batch data is of same size
        #cur_batch_data = np.zeros((BATCH_SIZE,NUM_POINT,TEST_DATASET.num_channel()))
        #cur_batch_label = np.zeros((BATCH_SIZE), dtype=np.int32)
        cur_batch_data = np.zeros((1, data.shape[0], data.shape[1]))
        cur_batch_label = np.zeros((self.BATCH_SIZE, self.NUM_POINT)).astype(np.int32)#np.zeros((1), dtype=np.int32)

        total_correct = 0
        total_seen = 0
        loss_sum = 0
        batch_idx = 0
        shape_ious = []
        total_seen_class = [0 for _ in range(self.NUM_CLASSES)]
        total_correct_class = [0 for _ in range(self.NUM_CLASSES)]

        batch_data, batch_label = data, 4#TEST_DATASET.next_batch(augment=False)
        bsize = batch_data.shape[0]

        #print('Batch: %03d, batch size: %d'%(batch_idx, bsize))
        # for the last batch in the epoch, the bsize:end are from last batch
        cur_batch_data[0:bsize,...] = batch_data

        cur_batch_label[0] = batch_label

        batch_pred_sum = np.zeros((self.BATCH_SIZE, self.NUM_CLASSES)) # score for classes

        loss_val = 0
        pred_val = np.zeros((self.BATCH_SIZE, self.NUM_POINT, self.NUM_CLASSES))

        for vote_idx in range(self.NUM_VOTES):
            # Shuffle point order to achieve different farthest samplings
            shuffled_indices = np.arange(self.NUM_POINT)
            np.random.shuffle(shuffled_indices)
            rotated_data = provider.rotate_point_cloud_by_angle(cur_batch_data[:, shuffled_indices, :],
                                                                vote_idx/float(self.NUM_VOTES) * np.pi * 2)
            feed_dict = {self.ops['pointclouds_pl']: rotated_data,
                         self.ops['labels_pl']: cur_batch_label,
                         self.ops['is_training_pl']: is_training}
            temp_loss_val, temp_pred_val = self.sess.run([self.ops['loss'], self.ops['pred']], feed_dict=feed_dict)
            loss_val += temp_loss_val
            pred_val += temp_pred_val
        loss_val /= float(self.NUM_VOTES)

        print pred_val

        """
        for i in range(cur_batch_size):
            cat = seg_label_to_cat[cur_batch_label[i,0]]
            logits = cur_pred_val_logits[i,:,:]
            cur_pred_val[i,:] = np.argmax(logits[:,seg_classes[cat]], 1) + seg_classes[cat][0]
        """


    def call(self,input):
        with tf.Graph().as_default():
            self.evaluate(data=input)
