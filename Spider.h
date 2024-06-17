// holds mob positions on LED strip
// -1 means despawned
int spider_pool[NUM_LEDS];

// placement of mobs on LED strip
// 1 = spider
// 2 = zombie
// 3 = creeper
// 4 = skeleton
// 5 = enderman
int mob_type_at_location[NUM_LEDS];

/** SPIDERS **/
// whether the spiders should spawn
bool spider_spawn = false;
// how often spiders spawn in seconds
int spawnRate = 1;
long timeLastSpawned = 0;

// whether the spiders are attacking or chillin
// this will depend on whether the lights are on or off in the room
bool spider_hostile = false;

// "seeing" distance
int spider_fov = 15;

// spider attack range
int spider_attack_range = 3;

// spider speed (LEDS per second)
int spider_speed = 2;
long timeLastMoved = 0;

// spider destinations (which LED position each spider is headed towards)
int spider_destinations[NUM_LEDS];

// spider directions
int spider_directions[NUM_LEDS];

int MAX_WANDER = 20;

int numSpidersSpawned = 0;
int numSpidersKilled = 0;
int spider_XP = 3;

/** ZOMBIES **/
bool zombie_spawn = false;
bool zombie_burn = false;

/** CREEPERS **/
bool creeper_spawn = false;

void spiderAttack()
{
  
}

void initSpiderData()
{
  for (int i = 0; i < NUM_LEDS; i++) {
    spider_pool[i] = -1;
    spider_destinations[i] = -1;
  }
}

void wander()
{
  for (int i = 0; i < NUM_LEDS; i++) {
    int distance;
    int new_direction;
    int new_pos;
    // if no spider exists
    if (spider_pool[i] == -1)
    {
      continue;
    }
    else if (spider_destinations[i] == -1 
    || spider_pool[i] == spider_destinations[i]) 
    {
      // get a new destination
      int j = 0;
      do {
        j++;
        distance = random(0, MAX_WANDER);
        new_direction = random(0, 3) - 1;
        new_pos = spider_pool[i] + new_direction * distance;
      } while ((new_pos < 0 || new_pos > NUM_LEDS));

      if (new_pos >= 0 && new_pos < NUM_LEDS) {
        
        spider_destinations[i] = new_pos;
        spider_directions[i] = new_direction;
      }
  
    }
    // head to destination
    else if (millis() - timeLastMoved > (1.0 / spider_speed) * 1000)
    {
      mob_type_at_location[spider_pool[i]] = 0;
      if (spider_directions[i] < 0)
      {
        spider_pool[i] -= 1;
      }
      else if (spider_directions[i] > 0)
      {
        spider_pool[i] += 1;
      }
      mob_type_at_location[spider_pool[i]] = 1;
    }
  }
  if (millis() - timeLastMoved > (1.0 / spider_speed) * 1000) {
    timeLastMoved = millis();
  }
}

void checkIfWithinRange(int pos, int range) {
  return abs(LED_pos - pos) <= range);
}

void killSpider(int id)
{
  if (spider_pool[id] == -1 || id < 0 || id >= NUM_LEDS)
    return;
  mob_type_at_location[spider_pool[id]] = 0; // FIX
  spider_pool[id] = -1;
  spider_destinations[id] = -1;
  spider_directions[id] = 0;
  if (numSpawned > 1) numSpawned = numSpawned - 1;
  numSpidersKilled++;
}

void spawnSpider()
{
  if (spider_spawn == true && numSpawned < NUM_LEDS)
  {
    long currentMillis = millis();
    if (currentMillis - timeLastSpawned < (1.0 / spawnRate) * 1000) {
      return;
    }
    
    int numSpiders = 1;
    for (int i = 0; i < NUM_LEDS; i++)
    {
      if (spider_pool[i] == -1)
      {
        int randomPos = random(0, NUM_LEDS);
        while (mob_type_at_location[randomPos] || LED_pos == randomPos) {
          randomPos = random(0, NUM_LEDS);
        }
        spider_pool[i] = randomPos;
        mob_type_at_location[randomPos] = i;
        numSpawned++;

        break;
      }
    }
    timeLastSpawned = millis();
  }
}
