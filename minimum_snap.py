import numpy as np
from cvxopt import matrix, solvers
import tkinter as tk
from rrt_line import *

"""
Reference:
https://blog.csdn.net/q597967420/article/details/76099491
https://blog.csdn.net/q597967420/article/details/77623235?spm=1001.2014.3001.5501
https://blog.csdn.net/q597967420/article/details/79031791?spm=1001.2014.3001.5501
https://blog.csdn.net/QW_sunny/article/details/79793889
"""


class minimum_snap:
    def __init__(self, path_list):
        self.T = 1  # 总时间
        self.t_to = []  # 到每个node的时间
        self.path_list = path_list  # size = len(node)*2
        self.path_list_x = [x[0] for x in self.path_list]
        self.path_list_y = [y[1] for y in self.path_list]
        self.Q_curvature = []
        self.Q_length = []
        self.p_x = []
        self.p_y = []
        self.M = []

        self.cal_time()
        self.def_Q_curvature()
        self.def_Q_length()
        self.p_constrain()
        self.M_constrain()
        self.G_constrain()

        # 这是最终离散化的结果
        self.x = []
        self.y = []

    def G_constrain(self):
        self.G = np.zeros([(len(self.t_to)-2)*2, (len(self.t_to)-1)*6])
        # print(len(self.G), len(self.G[0]))
        # 前面是<=，后面是>=
        for i in range(len(self.t_to)-2):
            self.G[i][i*6+0] = 1
            self.G[i][i*6+1] = self.t_to[i+1]
            self.G[i][i*6+2] = self.t_to[i+1]**2
            self.G[i][i*6+3] = self.t_to[i+1]**3
            self.G[i][i*6+4] = self.t_to[i+1]**4
            self.G[i][i*6+5] = self.t_to[i+1]**5
        for i in range(len(self.t_to)-2):
            for j in range((len(self.t_to)-1)*6):
                self.G[i+len(self.t_to)-2][j] = -self.G[i][j]

        # 找到bounds
        global lefts, rights, ups, downs
        lefts, rights, ups, downs = find_bounds(rrt_agent.col_map, path_list)
        self.hx = np.zeros([(len(self.t_to)-2)*2, 1])
        self.hy = np.zeros([(len(self.t_to)-2)*2, 1])
        # 前后面<=的，后面是>=的，注意正负号
        for i in range(len(lefts)-2):
            self.hx[i] = rights[i+1]
            self.hx[i+len(self.t_to)-2] = -lefts[i+1]
            self.hy[i] = ups[i+1]
            self.hy[i+len(self.t_to)-2] = -downs[i+1]

    # 计算到每一个node的时间self.t_to

    def cal_time(self):
        t_every = []
        Distance_all = 0
        for i in range(len(self.path_list) - 1):
            dis = np.sqrt((self.path_list[i + 1][1] - self.path_list[i][1])
                          ** 2 + (self.path_list[i + 1][0] - self.path_list[i][0]) ** 2)
            Distance_all += dis
        for i in range(len(self.path_list) - 1):
            dis = np.sqrt((self.path_list[i + 1][1] - self.path_list[i][1])
                          ** 2 + (self.path_list[i + 1][0] - self.path_list[i][0]) ** 2)
            t_every.append(dis / Distance_all * self.T)

        self.t_to.append(0)
        for i in range(len(t_every)):
            self.t_to.append(self.t_to[i] + t_every[i])  # 这里实际上有一种错位的思想

    # 计算当前index下的q
    def Q_temp(self, index, length=6):
        q_temp = np.ones([length, length]) * 0.0
        for i in range(length):
            for j in range(length):
                if i >= 3 and j >= 3:
                    q_temp[i][j] = (i + 1) * (i) * (i - 1) * (i - 2) * (j + 1) * (j) * (j - 1) * (j - 2) / (
                        i + j - 5) * (self.t_to[index] ** (i + j - 5) - self.t_to[index - 1] ** (i + j - 5))
        return q_temp

    # 计算全部的Q(目标函数中的Q)
    def def_Q_curvature(self):
        self.Q_curvature = np.zeros([k * (n + 1), k * (n + 1)])
        for i in range(k):
            q_temp = self.Q_temp(index=i + 1, length=6)
            num_q = i * (n + 1)
            for j in range(n + 1):
                for u in range(n + 1):
                    self.Q_curvature[num_q + j][num_q + u] = q_temp[j][u]  # 对角线那种添加
        # print('Q_curvature is done:')
        # print(self.Q_curvature.shape)
        # print('-' * 60)

    # 计算Q_length
    def def_Q_length(self):
        self.Q_length = np.zeros([k * (n + 1), k * (n + 1)])
        for i in range(k*(n+1)):
            if i==0:
                self.Q_length[i][i] = 1
                self.Q_length[i][i+1] = -1
            elif i ==k*(n+1)-1:
                self.Q_length[i][i-1] = -1
            else:
                self.Q_length[i][i] = 2
                self.Q_length[i][i-1] = -1
                self.Q_length[i][i+1] = -1

    # 起点终点的x v a的限制和中间点的x限制,这个是等式后面的部分.
    # 前3+k-1+3是对应的值，剩下的部分是0
    def p_constrain(self, vx0=0, ax0=0, vxk=0, axk=0, vy0=0, ay0=0, vyk=0, ayk=0):
        # define constrains: firstly equal constrains
        # p_x
        self.p_x = np.zeros([3 * k + 3, 1])
        # start node
        self.p_x[0][0] = self.path_list_x[0]
        self.p_x[1][0] = vx0
        self.p_x[2][0] = ax0
        # end node
        self.p_x[3][0] = self.path_list_x[k]
        self.p_x[4][0] = vxk
        self.p_x[5][0] = axk

        # p_y
        self.p_y = np.zeros([3 * k + 3, 1])
        # start node
        self.p_y[0][0] = self.path_list_y[0]
        self.p_y[1][0] = vy0
        self.p_y[2][0] = ay0
        # end node
        self.p_y[3][0] = self.path_list_y[k]
        self.p_y[4][0] = vyk
        self.p_y[5][0] = ayk

        # print('p is done')
        # print('k+5的值:', k + 5)
        # print('px的总长度:', len(self.p_x), 'py的总长度:',
        #       len(self.p_y), '4k+2的值:', 4 * k + 2)
        # print(self.p_x.shape)
        # print('-' * 60)

    # 和p相对应，建立x和y的M约束。注意实际上x和y的M是一样的
    def M_constrain(self):
        self.M = np.zeros([3 * k + 3, (n + 1) * k])
        # M1
        M1 = np.zeros([3 + 3, (n + 1) * k])
        # start node: p0,v0,a0
        for i in range(n + 1):
            M1[0][i] = self.t_to[0] ** i
            if i == n:
                continue
            M1[1][i + 1] = (i + 1) * self.t_to[0] ** i
            if i == n - 1:
                continue
            M1[2][i + 2] = (i + 2) * (i + 1) * self.t_to[0] ** i

        # end node: pk,vk,ak
        for i in range(n + 1):
            M1[3][i + (k - 1) * (n + 1)] = self.t_to[-1] ** i
            if i == n:
                continue
            M1[4][i + 1 + (k - 1) * (n + 1)] = (i + 1) * self.t_to[-1] ** i
            if i == n - 1:
                continue
            M1[5][i + 2 + (k - 1) * (n + 1)] = (i + 2) * \
                (i + 1) * self.t_to[-1] ** i

        # print('M1 is done')
        # print(M1.shape)
        # print('-' * 60)

        # define constrains: secondly equal constrains
        M2 = np.ones([3 * k - 3, (n + 1) * k]) * 0.0
        j = 0
        l = 0
        while l < 3 * (k - 1):
            for i in range(n + 1):
                M2[l][j * (n + 1) + i] = self.t_to[j + 1] ** i
                M2[l][j * (n + 1) + i + (n + 1)] = -self.t_to[j + 1] ** i
                if i == n:
                    continue
                M2[l + 1][j * (n + 1) + i + 1] = (i + 1) * \
                    self.t_to[j + 1] ** i
                M2[l + 1][j * (n + 1) + i + 1 + (n + 1)] = - \
                    (i + 1) * self.t_to[j + 1] ** i
                if i == n - 1:
                    continue
                M2[l + 2][j * (n + 1) + i + 2] = (i + 2) * \
                    (i + 1) * self.t_to[j + 1] ** i
                M2[l + 2][j * (n + 1) + i + 2 + (n + 1)] = - \
                    (i + 2) * (i + 1) * self.t_to[j + 1] ** i

            j += 1
            l += 3

        # print('M2 is done')
        # print(M2.shape)
        # print('-' * 60)

        # combine M1 and M2 to M
        # M=np.ones([4*k+2,(n+1)*k])*0.0
        for i in range(3 * k + 3):
            for j in range((n + 1) * k):
                if i <= 5:
                    self.M[i][j] = M1[i][j]
                else:
                    self.M[i][j] = M2[i - 6][j]

        # print('M is done')
        # print(self.M.shape)
        # print('-' * 60)

    # 计算结果

    def figure_out(self):
        q = np.zeros([len(self.Q_curvature), 1])
        q = matrix(q)
        Q_curvature = matrix(self.Q_curvature)
        Q_length = matrix(self.Q_length)
        w1 = 1
        w2 = 0
        M = matrix(self.M)
        G = matrix(self.G)

        p_x = matrix(self.p_x)
        hx = matrix(self.hx)
        print('-'*30)
        print(len(G))
        print(G)
        print('-'*30)
        print(len(hx))
        print(hx)
        result_x = solvers.qp(P=w1*Q_curvature+w2*Q_length, q=q, A=M, b=p_x, G=G, h=hx)

        p_y = matrix(self.p_y)
        hy = matrix(self.hy)
        result_y = solvers.qp(P=w1*Q_curvature+w2*Q_length, q=q, A=M, b=p_y, G=G, h=hy)

        lama_x = result_x['x']
        lama_y = result_y['x']

        # 这里的3个global只是为了画图
        global x
        global y
        x = []
        y = []
        # 调整后的waypoints
        global final_waypoints
        final_waypoints = []
        # 将每个片段离散成10个点，如果collision，则把两点的中点添加到list中
        for t_a in range(len(self.t_to) - 1):
            time_list = np.linspace(
                self.t_to[t_a], self.t_to[t_a + 1], Dis_num, endpoint=True)
            for j in time_list:
                m = np.zeros([len(lama_x), 1])
                m[0 + t_a * 6] = 1
                m[1 + t_a * 6] = j
                m[2 + t_a * 6] = j ** 2
                m[3 + t_a * 6] = j ** 3
                m[4 + t_a * 6] = j ** 4
                m[5 + t_a * 6] = j ** 5
                self.x.append(np.dot(np.transpose(m), lama_x)[0][0])
                self.y.append(np.dot(np.transpose(m), lama_y)[0][0])

                x.append(np.dot(np.transpose(m), lama_x)[0][0])
                y.append(np.dot(np.transpose(m), lama_y)[0][0])

                if j == 0 or j == time_list[-1]:
                    final_waypoints.append([np.dot(np.transpose(m), lama_x)[
                                           0][0], np.dot(np.transpose(m), lama_y)[0][0]])
                if int(np.dot(np.transpose(m), lama_x)[0][0]) >= len(rrt_agent.col_map) or int(np.dot(np.transpose(m), lama_x)[0][0]) < 0 or int(np.dot(np.transpose(m), lama_y)[0][0]) >= len(rrt_agent.col_map[0]) or int(np.dot(np.transpose(m), lama_y)[0][0]) < 0 or rrt_agent.col_map[int(np.dot(np.transpose(m), lama_x)[0][0])][int(np.dot(np.transpose(m), lama_y)[0][0])] > LEVEL:
                    print(np.dot(np.transpose(m), lama_x)[0][0])
                    print(np.dot(np.transpose(m), lama_y)[0][0])
                    print(rrt_agent.col_map[int(np.dot(np.transpose(m), lama_x)[
                          0][0])][int(np.dot(np.transpose(m), lama_y)[0][0])])
                    l = path_list.copy()
                    # 被插入的中间点
                    b1 = int((path_list[t_a][0] + path_list[t_a + 1][0]) / 2)
                    b2 = int((path_list[t_a][1] + path_list[t_a + 1][1]) / 2)
                    l.insert(t_a + 1, [b1, b2])
                    return False, l
        # print('-'*60)
        # print('x:')
        # print(self.x)
        # print('y:')
        # print(self.y)
        # print('-' * 60)
        return True, path_list


def draw():
    global x
    global y
    global final_waypoints
    for yyy in range(1, rrt_agent.height - 1):
        for xxx in range(1, rrt_agent.width - 1):
            if rrt_agent.col_map[xxx][yyy] > LEVEL:
                canvas.create_rectangle(xxx - 1, yyy - 1, xxx + 1,
                                        yyy + 1,
                                        fill='black')
    for i in range(len(path_list)):
        canvas.create_rectangle(path_list[i][0] - 2, path_list[i][1] - 2, path_list[i][0] + 2,
                                path_list[i][1] + 2,
                                fill='green')
    for i in range(len(final_waypoints)):
        canvas.create_rectangle(final_waypoints[i][0] - 2, final_waypoints[i][1] - 2, final_waypoints[i][0] + 2,
                                final_waypoints[i][1] + 2,
                                fill='red')
    
    global lefts, rights, ups, downs
    for i in range(len(lefts)):
        canvas.create_rectangle(lefts[i], downs[i], rights[i], ups[i])

    print("original waypoints: ", path_list)
    print("adjusted waypoints: ", final_waypoints)
    for i in range(len(x)-1):
        # print(x[i], y[i])
        canvas.create_line(int(x[i]), int(y[i]), int(
            x[i+1]), int(y[i+1]), fill='red')
        canvas.update()

def find_bounds(col_map, path_list):
    # 为每个路径点寻找边界
    temp_col_map = col_map.copy()
    lefts = []
    rights = []
    ups = []
    downs = []
    print('-'*30, 'bound,l,r,u,d')
    for i, point in enumerate(path_list):
        if i == 0:
            bound = find_bound(temp_col_map, path_list, point[0], point[1], 1)
        elif i == len(path_list)-1:
            bound = find_bound(temp_col_map, path_list, point[0], point[1], 1)
        else:
            bound = find_bound(temp_col_map, path_list, point[0], point[1], 0)
        print(point, '; ', bound)
        lefts.append(bound[0])
        rights.append(bound[1])
        ups.append(bound[2])
        downs.append(bound[3])
    return lefts, rights, ups, downs


def find_bound(col_map, path_list, px, py, endpoint=0):
    max_setoff = 10
    # 起点和终点 不允许移动
    if endpoint == 1:
        max_setoff = 0
    # 初始化bounds
    up = py
    down = py
    left = px
    right = px

    flag_l=1
    flag_r=1
    flag_u=1
    flag_d=1
    for i in range(max_setoff):
        if flag_l:
            for yi in range(int(round(down)),int(round(up+1))):
                if left-1<0:
                    break
                if col_map[int(round(left-1))][yi] > LEVEL:
                    # left -=0.5
                    flag_l = 0
                    break
            else:
                left -= 1
        
        if flag_u:
            for xi in range(int(round(left)),int(round(right+1))):
                if up+1>=len(col_map):
                    break
                if col_map[xi][int(round(up+1))] > LEVEL:
                    # up+=0.5
                    flag_u = 0
                    break
            else:
                up += 1
        
        if flag_r:
            for yi in range(int(round(down)), int(round(up+1))):
                if right+1>=len(col_map[0]):
                    break
                if col_map[int(round(right+1))][yi] > LEVEL:
                    # right+=0.5
                    flag_r = 0
                    break
            else:
                right += 1
        
        if flag_d:
            for xi in range(int(round(left)), int(round(right+1))):
                if down-1<0:
                    break
                if col_map[xi][int(round(down-1))] > LEVEL:
                    # down-=0.5
                    flag_d = 0
                    break
            else:
                down -= 1
  

    # 点的相对位置方向不应该发生改变
    for p in path_list:
        if p[0] == px and p[1] == py:
            continue
        if p[0] > left and p[0] <= px and p[1]>down and p[1]<=py:
            left = p[0]
            down = p[1]
        
        if p[0] > left and p[0] <= px and p[1]<up and p[1]>=py:
            left = p[0]
            up = p[1]
        
        if p[0]<right and p[0]>=px and p[1]>down and p[1]<=py:
            right = p[0]
            down = p[1]

        if p[0]<right and p[0]>=px and p[1]<up and p[1]>=py:
            right = p[0]
            up = p[1]

    left = max(0,left)
    right = min(len(col_map),right)
    up = min(len(col_map[0]),up)
    down = max(0,down)

    return [left, right, up, down]


# 先调用求解函数分别得到x和y的list
# 然后调用figure求解多项式
if __name__ == '__main__':
    rrt_agent = rrt()
    rrt_agent.init_map()
    path_list = rrt_agent.path_xy

    Dis_num = 20  # 每个多项式离散的点的个数
    LEVEL = 50  # 障碍物分界线
    n = 5

    # 这里是画图部分，无所谓
    window = tk.Tk()
    window.title('rrt')
    window.geometry('%dx%d' % (800, 900))
    canvas = tk.Canvas(window, bg='white', height=800, width=800)
    # b = tk.Button(window, text='draw', command=draw)
    canvas.place(x=0, y=0, anchor='nw')
    # b.place(x=400, y=850, anchor='nw')

    while True:
        k = len(path_list) - 1
        Fig_class = minimum_snap(path_list)
        is_success, path_list = Fig_class.figure_out()
        if is_success:
            break
    draw()

    window.mainloop()
