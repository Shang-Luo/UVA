import pygame
import math

"""
Rendering utilities only. This module must NOT import the algorithm implementation.
It exposes `init_display` and `draw_frame` to be used by the runner.
"""

def init_display(width=1000, height=700):
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 20)
    return screen, clock, font


def world_to_screen(p, scale=1.0, offset=(0, 0)):
    return (int(p[0]*scale + offset[0]), int(p[1]*scale + offset[1]))


def draw_frame(screen, font, state):
    """Draw the entire scene based on `state` dict.

    Required keys in state:
      - starts, goals, obstacles, planned_paths, drones
      - sim_running, btn_rect, btn_font
      - show_history, history_color, history_width
      - SCALE, OFFSET
    """
    SCALE = state.get("SCALE", 1.0)
    OFFSET = state.get("OFFSET", (0, 0))
    show_history = state.get("show_history", True)
    history_color = state.get("history_color", (50,120,255))
    history_width = state.get("history_width", 2)

    def _wts(p):
        return world_to_screen(p, SCALE, OFFSET)

    screen.fill((30,30,30))
    # draw starts/goals
    for s in state.get("starts", []):
        pygame.draw.circle(screen, (255,215,0), _wts(s), 4)
    for g in state.get("goals", []):
        pygame.draw.circle(screen, (255,0,0), _wts(g), 5)

    # draw obstacles
    for poly in state.get("obstacles", []):
        pts = [_wts(p) for p in poly]
        if len(pts) >= 3:
            pygame.draw.polygon(screen, (150,50,50), pts)
            pygame.draw.polygon(screen, (200,100,100), pts, 2)

    # draw planned paths
    for ppath in state.get("planned_paths", []):
        if len(ppath) >= 2:
            pts = [_wts(p) for p in ppath]
            pygame.draw.lines(screen, (255,215,0), False, pts, 2)

    # draw drones
    drones = state.get("drones", [])
    for i, drone in enumerate(drones):
        spos = _wts(drone.pos)

        # draw radar if provided
        show_radar = state.get("show_radar", False)
        radar_data = state.get("radar_data", [])
        if show_radar and i < len(radar_data):
            raw_readings, dirs = radar_data[i]
            for raw, ddir in zip(raw_readings, dirs):
                # endpoint in world coords
                endx = drone.pos[0] + ddir[0] * raw
                endy = drone.pos[1] + ddir[1] * raw
                end_s = _wts((endx, endy))
                pygame.draw.line(screen, (100,200,255), spos, end_s, 1)
                if raw < getattr(drone, "radar", 0):
                    pygame.draw.circle(screen, (255,80,80), end_s, 3)
                else:
                    pygame.draw.circle(screen, (120,120,120), end_s, 2)

        if show_history and len(drone.history) >= 2:
            hpts = [_wts(p) for p in drone.history]
            pygame.draw.lines(screen, history_color, False, hpts, history_width)

        r_world = getattr(drone, "radius", 6)
        r_screen = max(1, int(r_world * SCALE))
        pygame.draw.circle(screen, (50,220,120), spos, r_screen)

        label_size = max(10, int(r_world * 1.2))
        label_font = pygame.font.SysFont(None, label_size)
        txt = label_font.render(str(i), True, (0,0,0))
        txt_rect = txt.get_rect(center=(spos[0], spos[1]))
        screen.blit(txt, txt_rect)

        rtext = font.render(str(getattr(drone, "radar", "")), True, (200,200,200))
        rrect = rtext.get_rect(center=(spos[0], spos[1] + r_screen + 10))
        screen.blit(rtext, rrect)
    tris = state.get("tris", [])
    tri_color = (180, 180, 180)
    for tri in tris:
        # draw triangle edges
        pygame.draw.polygon(screen, tri_color, [tri[0], tri[1], tri[2]], 1)
    

    # draw button
    btn_rect = state.get("btn_rect")
    btn_font = state.get("btn_font")
    sim_running = state.get("sim_running", False)
    if btn_rect is not None and btn_font is not None:
        if sim_running:
            pygame.draw.rect(screen, (200,60,60), btn_rect)
            btn_text = btn_font.render("REPLACE", True, (255,255,255))
        else:
            pygame.draw.rect(screen, (60,180,80), btn_rect)
            btn_text = btn_font.render("START", True, (255,255,255))
        screen.blit(btn_text, (btn_rect.x + 18, btn_rect.y + 6))

    # draw FPS at top-right
    fps = state.get("fps", None)
    if fps is not None:
        fps_text = font.render(f"FPS: {fps:.1f}", True, (220,220,220))
        fps_rect = fps_text.get_rect()
        fps_rect.topright = (screen.get_width() - 8, 8)
        screen.blit(fps_text, fps_rect)

    # draw simulation time below FPS if provided (format time=M:SSs)
    sim_time = state.get("sim_time", None)
    if sim_time is not None:
        total_sec = int(sim_time)
        mins = total_sec // 60
        secs = total_sec % 60
        time_text = font.render(f"time={mins}:{secs:02d}s", True, (220,220,220))
        time_rect = time_text.get_rect()
        # place below FPS
        time_rect.topright = (screen.get_width() - 8, fps_rect.bottom + 6) if fps is not None else (screen.get_width() - 8, 28)
        screen.blit(time_text, time_rect)

    pygame.display.flip()
