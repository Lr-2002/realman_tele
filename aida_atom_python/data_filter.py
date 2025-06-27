#!/usr/bin/env python
# -*- coding: utf-8 -*-
class DataFilter:
    def __init__(self):
        self.filter_data_buf_ = [[0 for _ in range(200)] for _ in range(8)]
        self.filter_data_buf_sum_ = [0 for _ in range(8)]
        self.filter_data_buf_i_ = [0 for _ in range(8)]
        self.filter_data_buf_k_ = [0 for _ in range(8)]

    def Refilter(self, joint_id: int, in_data: float, in_count: int) -> float:
        if in_count == 0:
            return in_data

        j = 0
        re_average_data = 0.0

        self.filter_data_buf_k_[joint_id] += 1

        if self.filter_data_buf_k_[joint_id] > in_count:
            self.filter_data_buf_k_[joint_id] = in_count

            self.filter_data_buf_sum_[joint_id] = self.filter_data_buf_sum_[joint_id] - self.filter_data_buf_[joint_id][0]

            for j in range(1, in_count):
                self.filter_data_buf_[joint_id][j - 1] = self.filter_data_buf_[joint_id][j]
            self.filter_data_buf_[joint_id][in_count - 1] = in_data

            self.filter_data_buf_sum_[joint_id] += in_data

            re_average_data = self.filter_data_buf_sum_[joint_id] / in_count
        else:
            self.filter_data_buf_[joint_id][self.filter_data_buf_i_[joint_id]] = in_data

            self.filter_data_buf_i_[joint_id] += 1

            self.filter_data_buf_sum_[joint_id] += in_data

            re_average_data = self.filter_data_buf_sum_[joint_id] / self.filter_data_buf_i_[joint_id]

        return re_average_data

    def ResetFilterData(self):
        self.filter_data_buf_sum_ = [0 for _ in range(8)]
        self.filter_data_buf_i_ = [0 for _ in range(8)]
        self.filter_data_buf_k_ = [0 for _ in range(8)]