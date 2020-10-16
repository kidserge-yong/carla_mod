import pygame
import pygame_menu



def intro_menu(args):
    intro_width = 600   
    intro_height = 600

    actor_filter = [
        ('vehicle.*', 1),
        ('walker.*', 2)
    ]

    resolution_list =   [
                        ('1280×720', '1280×720'), 
                        ('1600×900', '1600×900'),
                        ('1920×1080', '1920×1080'),
                        ('2560×1440', '2560×1440')
                        ]


    pygame.init()
    surface = pygame.display.set_mode((intro_width, intro_height))
    pygame.display.set_caption("Option for Carla client.")

    def set_ip(ip):
        try:
            args.host = str(ip)
        except ValueError:
            print("ERROR: Please enter in type %s" %(type(args.host)))
        #print(args.host)

    def set_port(port):
        try:
            args.port = int(port)
        except ValueError:
            print("ERROR: Please enter in type %s" %(type(args.port)))
        #print(args.port)

    def set_actor_filter(actor_filter, index):
        try:
            args.filter = str(actor_filter[0])
        except ValueError:
            print("ERROR: Please enter in type %s" %(type(args.rolename)))
        #print(args.filter)

    def set_rolename(rolename):
        try:
            args.rolename = str(rolename)
        except ValueError:
            print("ERROR: Please enter in type %s" %(type(args.rolename)))

    def set_resolution(index, resolution):
        try:
            args.width, args.height = [int(x) for x in resolution.split('×')]
            args.res = '%sx%s' % (args.width, args.height)
        except ValueError:
            print("ERROR: Please enter in type %sx%s" %(type(args.width), type(args.height)))
        #print(args.width, args.height)

    def start():
        args.menu_flag = False
        #print("click start: ", args.menu_flag)

    args.menu_flag = True

    menu = pygame_menu.Menu(intro_width, intro_height, 'Welcome',
                        theme=pygame_menu.themes.THEME_BLUE)
    menu.add_text_input('Host IP: ', 
                        default = str(args.host), 
                        onchange=set_ip
                        )
    menu.add_text_input('Port: ', 
                        default = int(args.port), 
                        onchange=set_port
                        )
    menu.add_selector('Actor Filter: ', 
                        actor_filter,
                        onchange=set_actor_filter
                        )
    menu.add_text_input('Role: ', 
                        default = str(args.rolename), 
                        onchange=set_rolename
                        )
    menu.add_selector('Resolution: ', 
                        resolution_list,
                        default = 2, 
                        onchange=set_resolution
                        )
    menu.add_button('Play', 
                    start
                    )
    menu.add_button('Quit', pygame_menu.events.EXIT)
        
    while args.menu_flag is True:
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                exit()
        menu.update(events)
        menu.draw(surface)

        pygame.display.update()

    return args



def select_menu(args, actor_list):
    intro_width = 600   
    intro_height = 600

    pygame.init()
    surface = pygame.display.set_mode((intro_width, intro_height))
    pygame.display.set_caption("Select hero actor")

    def set_actor(type, actor_id):
        try:
            args.actor_id = actor_id
        except ValueError:
            print("ERROR: Please enter in type %sx%s" %(type(args.width), type(args.height)))
        #print(args.width, args.height)

    def start():
        args.menu_flag = False
        #print("click start: ", args.menu_flag)

    args.actor_id = actor_list[0][1]
    args.menu_flag = True

    menu = pygame_menu.Menu(intro_width, intro_height, 'Welcome',
                        theme=pygame_menu.themes.THEME_BLUE)
    menu.add_selector('Actor List: ', 
                        actor_list,
                        onchange=set_actor
                        )
    menu.add_button('Play', 
                    start
                    )
    menu.add_button('Quit', pygame_menu.events.EXIT)

    while args.menu_flag is True:
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                exit()
        menu.update(events)
        menu.draw(surface)

        pygame.display.update()

    return args




