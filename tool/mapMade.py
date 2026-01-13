import pygame
import json
import os
import sys

# 简单的 Pygame 地图编辑器
# 操作说明（程序内也会显示）:
# p 切换到障碍物模式 (SET=1)
# o 切换到起终点模式 (SET=2)
# 鼠标左键: 在障碍物模式下添加点（若尚未开始则开始新多边形），在起终点模式下依次添加起点/终点
# 鼠标右键: 在障碍物模式下结束当前多边形（如果点数>=3，则保存为障碍物）
# Ctrl+Z: 在障碍物模式下删除当前临时多边形的最后一点
# Ctrl+S: 保存，弹出命令行输入文件名前缀 A，保存为 A_M.json (障碍物) 和 A_S.json (起终点)


def load_map_size():
    # 尝试从 run.py 中读取宽高常量 MAP_W / MAP_H 或者从配置读取
    # 回退默认大小
    try:
        import run as r
        w = getattr(r, 'MAP_W', None)
        h = getattr(r, 'MAP_H', None)
        if w and h:
            return int(w), int(h)
    except Exception:
        pass
    return 800, 600


def get_chinese_font(size):
    # 尝试常见的中文字体名或系统字体文件路径，返回第一个可用字体
    try:
        names = [
            'microsoftyahei', 'msyh', 'simhei', 'simsun', 'arialuni', 'arialunicode'
        ]
        for name in names:
            path = pygame.font.match_font(name)
            if path:
                return pygame.font.Font(path, size)
        # 尝试 Windows 字体目录常见文件
        windir = os.environ.get('WINDIR', r'C:\Windows')
        candidates = ['msyh.ttf', 'msyh.ttc', 'simhei.ttf', 'simsun.ttc']
        for fname in candidates:
            p = os.path.join(windir, 'Fonts', fname)
            if os.path.exists(p):
                return pygame.font.Font(p, size)
    except Exception:
        pass
    return pygame.font.SysFont(None, size)


def save_files(prefix, obstacles, positions):
    base = os.path.join('set', prefix)
    # 障碍物保存为 A_M.json（多边形数组，与 set/obstacles.json 格式一致）
    mpath = base + '_M.json'
    # 起终点保存为 A_S.json（每项为 {"start":[x,y], "goal":[x,y], "radius":int, "radar_range":int}）
    spath = base + '_S.json'
    os.makedirs('set', exist_ok=True)
    # 保存障碍物：确保是列表的多边形点对
    with open(mpath, 'w', encoding='utf-8') as f:
        json.dump(obstacles, f, ensure_ascii=False, indent=2)

    # positions 存储为对象数组，包含 start/goal 和默认 radius/radar_range
    pos_out = []
    for p in positions:
        # 支持两种内部表示： [sx,sy,ex,ey] 或者 {'start':..., 'goal':...}
        if isinstance(p, dict):
            # 假设已是正确格式
            entry = {
                'start': p.get('start'),
                'goal': p.get('goal'),
                'radius': p.get('radius', 12),
                'radar_range': p.get('radar_range', 25)
            }
        else:
            # 列表格式
            try:
                sx, sy, ex, ey = p
            except Exception:
                # 跳过格式错误项
                continue
            entry = {'start': [int(sx), int(sy)], 'goal': [int(ex), int(ey)], 'radius': 12, 'radar_range': 25}
        pos_out.append(entry)

    with open(spath, 'w', encoding='utf-8') as f:
        json.dump(pos_out, f, ensure_ascii=False, indent=2)
    print(f"Saved: {mpath}, {spath}")


def main():
    pygame.init()
    MAP_W, MAP_H = load_map_size()
    screen = pygame.display.set_mode((MAP_W, MAP_H))
    pygame.display.set_caption('Map Maker')

    clock = pygame.time.Clock()

    # 全局变量 set
    SET = 0

    obstacles = []  # 已确认的多边形：[[[x,y],...], ...]
    temp_polygons = []  # 临时多边形列表（允许同时存在多个未完成项）
    positions = []  # 起终点数组：[[start_x,start_y, end_x,end_y], ...]
    temp_pos = []  # 临时存放起/终点，按两点一组

    font = get_chinese_font(24)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                mods = pygame.key.get_mods()
                if event.key == pygame.K_p:
                    SET = 1
                elif event.key == pygame.K_o:
                    SET = 2
                elif event.key == pygame.K_s and (mods & pygame.KMOD_CTRL):
                    # 保存
                    pygame.display.iconify()
                    prefix = input('保存文件名前缀 A: ').strip()
                    if prefix:
                        save_files(prefix, obstacles, positions)
                    pygame.display.set_mode((MAP_W, MAP_H))
                elif event.key == pygame.K_z and (mods & pygame.KMOD_CTRL):
                    if SET == 1:
                        # 删除临时多边形数组的最后一个点
                        if temp_polygons:
                            last = temp_polygons[-1]
                            if last:
                                last.pop()
                                if not last:
                                    temp_polygons.pop()
                elif event.key == pygame.K_ESCAPE:
                    running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                x, y = event.pos
                if event.button == 1:  # 左键
                    if SET == 1:
                        # 如果没有正在编辑的临时多边形，开始一个
                        if not temp_polygons or (temp_polygons and temp_polygons[-1]):
                            # 如果最后一个临时多边形为空则复用，否则新建
                            pass
                        if not temp_polygons or (temp_polygons and temp_polygons[-1] == []):
                            temp_polygons.append([[x, y]])
                        else:
                            temp_polygons[-1].append([x, y])
                    elif SET == 2:
                        # 起终点模式，依次添加起点和终点
                        temp_pos.append([x, y])
                        if len(temp_pos) == 2:
                            # 合并为一组并加入 positions
                            sx, sy = temp_pos[0]
                            ex, ey = temp_pos[1]
                            positions.append([sx, sy, ex, ey])
                            temp_pos = []
                elif event.button == 3:  # 右键
                    if SET == 1:
                        # 结束当前多边形
                        if temp_polygons:
                            last = temp_polygons[-1]
                            if len(last) >= 3:
                                obstacles.append(last[:])
                            temp_polygons.pop()

        # 绘制
        screen.fill((30, 30, 30))

        # 已保存的障碍物（红色半透明填充）
        for poly in obstacles:
            if len(poly) >= 3:
                pygame.draw.polygon(screen, (160, 30, 30), poly)
                pygame.draw.polygon(screen, (220, 80, 80), poly, 2)

        # 临时多边形（蓝色线和点）
        for poly in temp_polygons:
            if poly:
                if len(poly) >= 2:
                    pygame.draw.lines(screen, (0, 150, 255), False, poly, 2)
                for p in poly:
                    pygame.draw.circle(screen, (0, 200, 255), p, 4)

        # positions: 起点绿色，终点黄色，连线
        for idx, pos in enumerate(positions):
            sx, sy, ex, ey = pos
            pygame.draw.circle(screen, (0, 200, 0), (sx, sy), 6)
            pygame.draw.circle(screen, (220, 200, 0), (ex, ey), 6)
            pygame.draw.line(screen, (180, 180, 180), (sx, sy), (ex, ey), 2)
            # 索引标记
            txt = font.render(str(idx), True, (255, 255, 255))
            screen.blit(txt, (sx + 8, sy - 8))

        # 临时起点（若存在）显示
        for i, p in enumerate(temp_pos):
            color = (0, 200, 0) if i == 0 else (220, 200, 0)
            pygame.draw.circle(screen, color, p, 6)

        # 右上角显示模式
        mode_text = '模式: 未选' if SET == 0 else ('模式: 障碍物' if SET == 1 else '模式: 起终点')
        mt = font.render(mode_text, True, (255, 255, 255))
        screen.blit(mt, (10, 10))

        instr = '按 p: 障碍物, o: 起终点, Ctrl+S: 保存, Ctrl+Z: 撤销, 右键: 结束多边形'
        it = font.render(instr, True, (200, 200, 200))
        # 向上移动10像素
        screen.blit(it, (10, MAP_H - 34))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == '__main__':
    main()
