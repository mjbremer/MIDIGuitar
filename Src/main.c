/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
uint8_t pData[DMA_BUFFER_SIZE_BYTES];
int32_t processingData[BUFFER_SIZE];// = {-487022848,-513732864,-540231680,-567246336,-593612800,-620163328,-646993152,-672968960,-699356160,-726920704,-749721600,-733488640,-704489472,-678922240,-651683072,-625559040,-598586368,-571868672,-545039872,-518297088,-491078400,-464421888,-437412607,-410904064,-384335360,-330573056,-304283392,-277671424,-250823168,-223814144,-197261056,-170726656,-144517120,-118011648,-91072256,-64576256,-37900288,-11190784,15619840,42216448,68780800,95213312,121977088,148553472,175224832,201925888,228478976,255147777,281490944,307898368,334635264,361067776,387730688,440789760,440789760,494080512,520320512,547071488,573482240,600766464,626899201,653870592,679861760,707541504,733259008,761784064,767209472,739220992,713837056,686087424,660183552,633121536,606394624,579489792,552681472,526541824,499430656,472923904,446081536,419562752,366722304,339909120,313153536,286387968,259760640,233046784,206494464,180008192,153457408,126905856,100505856,73704960,47095296,20617216,-6043136,-32336640,-59225088,-85776128,-112188928,-138718464,-165440256,-192070656,-218881024,-245457920,-271856384,-298547456,-351668224,-378286080,-404552704,-431442688,-458481919,-485081856,-511339264,-537619200,-564012800,-591148544,-617729024,-643237376,-669836032,-696873984,-723729152,-748786176,-736042240,-706607616,-680897792,-653140224,-627027200,-599875072,-573191168,-546683648,-520305152,-466097920,-439814400,-413481728,-386570752,-359652608,-333313536,-306941696,-280327424,-252807936,-226063872,-199732992,-173015808,-146288896,-119727872,-93149696,-66527744,-39595520,-12877568,13773568,40405248,66931968,93603072,120090880,146716672,173069824,199566080,226188288,279461632,306242048,332818432,359264512,385853952,412632576,438893824,465451008,491793920,518634496,545030144,571236608,598165504,624946944,651417857,677900800,705254144,730800896,759533056,770287616,743991552,717874688,689844224,663384832,636547584,610139904,583058432,555904512,502391040,475713792,448760320,422312960,395390720,368912897,342337024,315680512,288908544,262446848,235703808,208829696,182356480,155460352,128801024,102181632,75637248,48930304,22517504,-4083712,-30610944,-57317120,-83851264,-110395392,-136948480,-163643904,-217010432,-243678720,-270572032,-296914944,-323575551,-350040320,-376753664,-403202560,-430006784,-456615168,-483096320,-509985024,-536326656,-563196416,-589412608,-616061952,-643144192,-669338624,-696137984,-722950400,-747771904,-735964928,-706417664,-680728576,-653508352,-627342080,-573061632,-546475264,-519006208,-492767488,-465728000,-439155712,-412504576,-385731072,-359296512,-332485120,-306564096,-279213568,-252351744,-225705984,-198735104,-172548607,-145934336,-119645696,-92769536,-65979648,-39346432,-12388864,14326528,41062656,67510528,93934848,120542208,173581312,200061441,226842112,253595136,280254720,306957056,333592832,360350208,387283456,413772288,440590336,467002880,493566720,519876608,546666496,573316096,599393792,626193408,652667392,679286528,706938624,731804160,761199872,770301440,743736832,689887744,689887744,637216768,610545664,583332096,556675840,529647104,502990080,476261632,449892864,423332864,396497408,369935616,343339008,316588288,289914368,263360256,236559872,210115840,183496960,156889344,130406912,103838720,76969728,50217472,23651840,-2843903,-29520896,-56110592,-109436928,-136193792,-162966016,-189494528,-216296959,-242727680,-269592576,-295753472,-322570752,-349481984,-376637696,-402944512,-429700608,-456849664,-483590912,-510251008,-536205568,-563217664,-589477120,-616040448,-642091520,-668443648,-695559424,-721630464,-735400192,-706842112,-680581376,-653490432,-626346240,-599282176,-573261056,-545853184,-518977280,-492078848,-465889536,-439189248,-412619008,-385364224,-358955264,-332481024,-305737472,-279026944,-252466176,-225531904,-198945792,-172491008,-145791744,-119747072,-92887296,-39566592,-39566592,13636608,40293888,67175169,93784832,120273408,147095552,173569536,200270848,226928640,253241344,279825152,306618880,332945152,359901440,386326784,413026560,439334912,466028032,492457984,519252224,545985024,572441857,599126016,625485568,652818432,678471936,706171392,731266304,771238400,744934912,718748416,691009536,665061888,637500672,610986240,584455680,557536000,530880768,504217600,477296640,450546688,423991552,397533952,371051008,344225024,317527296,290888960,264304384,237423872,210603776,183795456,157444864,130701312,104250880,50982656,24247040,-2336000,-28830720,-55363071,-82028544,-108483584,-135169280,-161702912,-188355328,-214713600,-241555200,-268597248,-295228416,-321328896,-348292608,-374788096,-401603839,-428616960,-455213312,-481466368,-508191232,-534899712,-561707263,-588631296,-641360896,-667272704,-694441984,-720326912,-746189056,-735622400,-706675200,-681279744,-652752128,-627271936,-600020480,-573674496,-547119872,-520338944,-492996096,-466713344,-439933952,-413260288,-386406656,-359736576,-333156608,-306072320,-279869696,-252922624,-226028288,-198886144,-145729024,-145729024,-92184320,-65382144,-38908928,-12112384,14487552,41083136,68027904,94527744,121120768,147854848,174448640,200869120,227351808,253923328,280532737,307203328,333481728,360226816,387031040,413363200,440222465,466692352,493454848,520065792,546742016,573361664,600107264,626967296,654139392,707323648,732894720,761842944,771019008,744026624,717961984,690081024,663945984,637379328,610987776,583954432,557153536,530555648,503569920,477260288,450423552,423636993,397359616,370612224,344143872,317652992,290865152,264297984,237619968,211139072,157719040,131102208,104386560,77869568,51062016,24473856,-2114304,-28589568,-55172096,-81737216,-108253952,-134986752,-161360640,-188527104,-214907904,-241266176,-268006143,-294601472,-321230592,-347422976,-374386688,-401061120,-427733503,-454630912,-481213696,-534323712,-560676608,-587680512,-614081280,-640650751,-667237632,-694501120,-720020480,-746909696,-735450112,-706779391,-681139200,-653453312,-627464192,-599944448,-573779712,-546294016,-520174848,-492781824,-466338304,-440138496,-413374720,-386536448,-359764992,-333600768,-306497792,-252868096,-226056448,-199424255,-172843520,-146249728,-119723520,-93194496,-66047232,-39345408,-12444160,14289920,40857856,67500288,93962496,120532992,147052289,173682944,200240640,226757120,253483520,280162560,306814977,333192704,360062720,386498304,413204224,439995904,466967552,493932800,547431680,547431680,600983296,627551232,654191872,680454400,707889152,733250304,761679104,771677696,744726016,718321408,691228672,664170240,637750784,611235328,584427264,557609984,531327488,504700160,477785088,451266560,424577280,398033664,371334400,344825856,317915904,291222528,238024448,211532032,184658945,157861888,131290368,104728064,78052352,51448576,24644352,-1890560,-28477440,-54915840,-81813504,-108327168,-134702592,-161643008,-188299520,-215080704,-241564672,-268269824,-294905088,-321377280,-348361472,-374889728,-401718784,-428369664,-455380736,-508515327,-535324928,-561778176,-588904960,-614883584,-641901568,-668024320,-694458112,-721198080,-746273792,-733697024,-704447232,-679250688,-651674368,-625143808,-598522368,-571694336,-545225728,-517774336,-491462912,-464599808,-438035712,-411625472,-384309248,-358202624,-331170304,-277546752,-250577664,-224117504,-197126144,-170908160,-144062720,-117772288,-91025408,-64156672,-37735936,-11202560,15298816,42015232,68656640,95525120,122070016,148584960,175252224,202000128,228604672,255377920,282192896,308678144,335410688,361931264,388584960,415453440,441983488,494987264,521955328,548851456,575319552,601871104,628109312,654992640,681354240,708369152,733863424,763122688,771842048,744467456,718307584,691234304,664338688,637614080,610485248,583714048,557130240,530711808,503755776,476773120,450221056,423586048,396990208,343536128,316867072,290191104,263560192,237070080,210463232,183661824,156956160,130171648,103854336,77129472,50318848,23267840,-3319040,-29840128,-56530176,-82924032,-109529600,-136122880,-162916608,-189474816,-216209664,-242564608,-269108736,-295472640,-322428160,-375510271,-375444736,-428755456,-456189440,-482149376,-508892672,-535151616,-561598720,-588523776,-615043072,-641493760,-667909376,-694727424,-721366784,-746724096,-733036288,-704043520,-677711360,-650753792,-624114687,-597164800,-570667520,-544208384,-517003008,-490612736,-464102912,-437096704,-383524096,-357184000,-330364672,-304007680,-277113087,-250019072,-223379712,-196620032,-170108160,-143401984,-116958208,-89879552,-63317248,-36778752,-10216704,16294912,43127808,69517057,96214528,123022336,149625088,176303616,203021824,229594369,256307200,282734080,309310208,362675968,362675968,416113664,442762752,469383936,495777536,522618624,549178624,576051968,602278144,628528640,655846912,682301696,709065728,734405120,763724800,771504896,744308480,718471424,691220992,664576000,637826816,611158784,584409088,557550848,530964224,503922176,477298176,450669056,397543680,370502656,344121088,317271808,290763776,264281344,237944576,211369216,184405248,157999360,131337472,104692481,77996288,51209216,24344320,-2604032,-29030144,-55702784,-82177792,-108840960,-135352320,-161840896,-188494336,-215211008,-241595648,-268129792,-321538560,-347769088,-374215424,-400937984,-427955456,-454676736,-480855040,-507680512,-533785856,-560668160,-587699968,-614042368,-640711168,-667186176,-694025728,-719442688,-745643008,-733192448,-704620288,-678307584,-651216896,-624830720,-597393152,-571509248,-544712192,-490799872,-464490752,-437555200,-410855168,-383846144,-357323776,-330726912,-304102400,-277753600,-250780416,-223477504,-196781312,-170123008,-143197696,-116683008,-89923072,-63225599,-36553984,-9976576,16712192,43404800,70093056,96654080,123260161,149884672,176692992,203078400,229584384,282723840,309362688,335711232,362578944,389093120,415933696,442479360,469667840,496246784,522671104,549594624,576107520,602714624,629192192,656283136,682559744,709491712,734577920,764124928,771944192,743878400,717803520,691112448,665003008,637480704,610960128,583954432,530801664,530801664,477370880,450632448,423884800,397059584,370536448,343763712,316918784,290187264,263723520,237116416,210591488,184052480,157442816,130902016,104003072,77093888,50311936,23629568,-2879488,-29523712,-56193280,-82917888,-109489664,-135556608,-162393344,-189091840,-242724608,-242724608,-295901952,-322420224,-349401088,-375571712,-402743808,-429014528,-455860736,-482823168,-509932032,-536210176,-563128064,-589256448,-615517695,-642223104,-668401152,-695182080,-721727488,-746368000,-730432512,-702412544,-676399104,-648951296,-622799872,-595590144,-569450496,-542495744,-488874752,-462454272,-435949056,-409300480,-382054144,-355611903,-329282304,-302306816,-275267072,-248294144,-221322240,-194593536,-167986688,-141240832,-114271744,-87481344,-60837376,-34454784,-7553280,19237632,45643264,72003584,98491904,125309696,151714560,178405888,231347200,257818880,284247552,310864640,337421056,363990528,390681600,417382912,444289792,470693632,497474304,523766016,550671360,577313024,604114432,630783488,657575168,683941632,711012608,736566784,765181696,771485184,743001600,717387776,690027008,664160512,636690432,610158848,556184576,529725953,502527488,476103168,449123072,422331648,395506432,368911104,342149888,315483136,288749824,262269952,235762432,209020416,182405888,155744256,129166592,102364160,75351808,48588544,21782528,-4881152,-31566848,-58367232,-84540672,-111265024,-137777151,-164477440,-217952512,-244453632,-271033344,-297371392,-323706112,-350578432,-377706496,-404559872,-430798592,-458073344,-484460288,-511300608,-537461760,-564137984,-590977792,-617883904,-643968512,-671048448,-697150208,-724509440,-746609920,-728280320,-700653056,-674299904,-647384832,-594300416,-567602944,-540278784,-514030080,-486688000,-460389632,-433350144,-406829824,-379624704,-353260544,-327102208,-300772096,-273656832,-246774272,-219796480,-192914176,-166107904,-139436032,-113101056,-86224896,-59438592,-32954880,-6081024,20684033,47319808,73888768,100468992,127173888,180484864,207012096,233472512,260015872,286639872,313221632,339899648,366673664,393409536,419984384,446771200,473480960,500077824,526319360,553414657,579788800,606720768,633159168,659998720,686348800,713614848,738898432,767358464,770448384,741712896,715962112,688171265,635215104,635215104,581676032,554266368,527298304,500451072,473896193,447349504,420576512,394069248,367357184,340733440,313841408,287024640,260710656,234167296,207685888,181015040,154446592,128000512,101176576,74575872,47821056,21081600,-5650432,-32294143,-59167232,-85618176,-138363648,-164797696,-191435520,-218043392,-244905984,-271339264,-298127872,-324049664,-351138048,-378111232,-404668672,-431560704,-457856512,-484858880,-511921408,-538717440,-564718592,-591641344,-617835776,-644878848,-671512064,-697547264,-725548543,-747087872,-698968064,-673597184,-646464000,-619468288,-592427519,-565992448,-539638272,-512826624,-485472256,-459386112,-432791808,-406259456,-379119872,-352594176,-325962752,-299140352,-272694783,-245586944,-219000576};
//int32_t sendingData[DMA_HALFBUF_SIZE_STEREO_SAMPLES];
// each type is a sample, so the number of samples in a half buffer is correct

static volatile uint8_t debug_toggle;
static volatile uint8_t* validpData;
static volatile uint8_t processingFirstHalf, processingSecondHalf;

state_t state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */




const unsigned char UARTtest[] = "The human torch was denied a bank loan\n";


void fillProcessingBuffer()
{
	int i;

	for (i = 0; i < BUFFER_SIZE; i++) {
		int32_t temp = 0;
		temp |= validpData[(8*i) + 1] << 24;
		temp |= validpData[(8*i)] << 16;
		temp |= validpData[(8*i) + 3] << 8;
		temp |= validpData[(8*i) + 2] << 0;

		processingData[i] = temp;
	}
}

void sendMIDI(uint8_t*msg, size_t size)
{
	HAL_UART_Transmit(&huart5, msg, size,10000000U);
}

void MIDIon(uint8_t channel, uint8_t note, uint8_t velocity)
{
	uint8_t msg[3];

	msg[0] = 0x90 | (channel & 0x0F);
	msg[1] = note;
	msg[2] = velocity & 0x7F;

	sendMIDI(msg, 3);
}

void MIDIoff(uint8_t channel, uint8_t note, uint8_t velocity)
{
	uint8_t msg[3];

	msg[0] = 0x80 | (channel & 0x0F);
	msg[1] = note;
	msg[2] = 0;//velocity & 0x7F;

	sendMIDI(msg, 3);
}

int32_t detect_attack() {

	int32_t threshold = (INT32_MAX / 4);

	for (int i = 0; i < BUFFER_SIZE; i++) {
		if (processingData[i] > 0) {
			if (processingData[i] > threshold) return i;
		} else {
			if (processingData[i] < -threshold) return i;
		}
	}
	return -1;
}

int32_t detect_release() {
	return -1;
}


float ProcessActiveBufferOld()
{

	fillProcessingBuffer();
	//MIDIoff(0, (uint8_t)cur_midi, 0x7F);
	state.cur_midi = detect_pitch(processingData);
	MIDIon(0, (uint8_t)state.cur_midi, 0x7F);
	return 0;
}

uint8_t ProcessActiveBuffer()
{
	int32_t attack;
	int32_t release;
	uint8_t new_midi;
//	fillProcessingBuffer();
//	attack = detect_attack();
//	if (attack != -1) {
//		MIDIoff(0, state.cur_midi, 0x7F);
//		MIDIon(0, state.cur_midi, 0x7F);
//	}
//
//	return attack;

	if (state.action == WAIT) {
		fillProcessingBuffer();
		for (int i = 0; i < BUFFER_SIZE; i++) state.AD_buffer[ATTACK_TIME + i] = processingData[i];
		attack = detect_attack();
		if (attack != -1) {
			state.action = FILL_PD_BUFFER;
			state.PD_pointer = 0;
		} else {
			release = detect_release();
			if ((state.playing == 1) && (release != -1)) {
				state.playing = 0;
				MIDIoff(0, state.cur_midi, 0x7F);
			}
			for (int i = 0; i < ATTACK_TIME; i++) state.AD_buffer[i] = state.AD_buffer[AD_BUFFER_SIZE - ATTACK_TIME + i];
		}
	}

	if (state.action == FILL_PD_BUFFER) {
		for (int i = ((attack == -1) ? 0 : attack); i < BUFFER_SIZE; i++) {
			state.PD_buffer[state.PD_pointer++] = processingData[i];
			if (state.PD_pointer == PD_BUFFER_SIZE) break;
		}

		if (state.PD_pointer == PD_BUFFER_SIZE) {
			new_midi = detect_pitch(state.PD_buffer);
			if (state.playing == 1) {
				MIDIoff(0, state.cur_midi, 0x7F);
			}
			state.cur_midi = new_midi;
			state.playing = 1;
			MIDIon(0, state.cur_midi, 0x7F);
			state.action = WAIT;
			//HAL_Delay(1000);
			return 1;
		}
	}

	return 0;
}

void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
	processingFirstHalf = 1;
	debug_toggle = 0;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	processingSecondHalf = 1;
	debug_toggle = 1;
}

void ProcessingLoop_NoAttack()
{
	  // General Form:
	  	  // Wait for half interrupt
	  	  // Process first half
	  	  // Output MIDI
	  	  // If full interrupt is already raised, error out
	  	  // Clear half interrupt flag

	  	  // Wait for full interrupt
	  	  //Process second half
	  	  // Output MIDI
	  	  // If half interrupt flag is already raised, error out
	  	  // Clear full interrupt flag


	  while (!processingFirstHalf);
	  validpData = pData;
	  ProcessActiveBuffer();

	  if (processingSecondHalf)
		 Error_Handler();

	  processingFirstHalf = 0;


	  while (!processingSecondHalf);

	  validpData = &pData[DMA_BUFFER_SIZE_BYTES/2];
	  ProcessActiveBuffer();

	  if (processingFirstHalf)
		  Error_Handler();

	  processingSecondHalf = 0;
}

void ProcessingLoop_DumpandGuess()
{
		uint8_t chump_check;
		HAL_Delay(1000);
		chump_check = debug_toggle;
		validpData = chump_check ? &pData[DMA_BUFFER_SIZE_BYTES/2] : pData;
		fillProcessingBuffer();

		// If toggle chained, we were writing over while copying. Whoops! Try again.
		if (debug_toggle != chump_check)
			return;


	  // Transmit processing buffer
	  HAL_UART_Transmit(&huart4, (uint8_t*)processingData, BUFFER_SIZE * sizeof(int32_t), 10000000U);


	  state.cur_midi = detect_pitch(processingData);
	  // Send MIDI message, but over debug uart
		uint8_t msg[3];
		msg[0] = 0x90;
		msg[1] = state.cur_midi;
		msg[2] = 0x7F;
	  HAL_UART_Transmit(&huart4, msg, 3,10000000U);


}

void ProcessingLoop()
{
	  // General Form:
	  	  // Wait for half interrupt
	  	  // Process first half
	  	  // If full interrupt is already raised, error out
	  	  // Clear half interrupt flag

	  	  // Wait for full interrupt
	  	  //Process second half
	  	  // If half interrupt flag is already raised, error out
	  	  // Clear full interrupt flag


	// Assume i our maximum allowable response time after an attack, and j is our minimum necessary buffer length to achieve our desired accuracy
	// y = ceil((j+k-m)/n)
	// Processing:
	// Get n ms buffer by swizzling
	// Append this buffer to the last m ms of the last buffer (where m is the maximum attack time of our instrument)
	// Perform attack detection on this m+n ms buffer (Assume this takes x ms) (Note: I believe x is O(m+n))
	// If an attack is found at position k ms, it has been (x+m+n-k) ms since the attack.
		// We have taken (x+m+n-k) ms at this point
	// We now need a j ms buffer to perform pitch detection on. If (m+n-k) < j, we must wait for more buffers (y total) to come in. This will take (y-1)n time.
		// Once we get all of these buffers, we have taken x + m + yn - k ms.
	// Now we have the necessary data, we can perform pitch detection on this array. Assume this takes z time. (Note: z is O(jlogj))
		// Time is now x+yn+z+m-k
	// If a note is currently playing, send a MIDI off signal for that note
	// Then, finally, send the MIDI on for the detected note (Assume both these transmissions take t ms, so UART transmission takes 2t ms total)
	// Time is now x+yn+z+m-k + 2t, which must be less than or greater than i

	uint8_t got_attack;


	  while (!processingFirstHalf);
	  validpData = pData;
	  got_attack = ProcessActiveBuffer();

	  if (processingSecondHalf && !got_attack)
		 Error_Handler();

	  processingFirstHalf = 0;
	  //processingSecondHalf = 0;

	  while (!processingSecondHalf);

	  validpData = &pData[DMA_BUFFER_SIZE_BYTES/2];
	  got_attack = ProcessActiveBuffer();

	  if (processingFirstHalf && !got_attack)
		  Error_Handler();

	  processingSecondHalf = 0;
	  //processingFirstHalf = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  //HAL_Delay(10000);
  processingFirstHalf = 0;
  processingSecondHalf = 0;
  debug_toggle = 0;
  state.cur_midi = 69;
  state.PD_pointer = 0;
  state.action = WAIT;
  state.playing = 0;
  for (int i = 0; i < AD_BUFFER_SIZE; i++) state.AD_buffer[i] = 0;
  for (int i = 0; i < PD_BUFFER_SIZE; i++) state.PD_buffer[i] = 0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(400);


//  HAL_Delay(5000);
//  meme();
//  while(1);
  if (init_vars() != 0)
	  Error_Handler();
  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*)pData, DMA_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  //ProcessingLoop_DumpandGuess();

	 ProcessingLoop();
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }

  // ---Function Call Graveyard---
  //    //Delay 1500ms
  //	  HAL_Delay(1500);
  //	  // Stop DMA
  //	  HAL_I2S_DMAStop(&hi2s2);
  //    // Transmit processing buffer
  //	  HAL_UART_Transmit(&huart4, (uint8_t*)processingData, DMA_HALFBUF_SIZE_STEREO_SAMPLES * 4U, 10000000U);
  //	  // Send Current MIDI note
  //	  MIDIon(0, (uint8_t)cur_midi, 0x7F);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */
  hi2s2.hdmarx = &hdma_spi2_rx;
  HAL_I2S_MspInit(&hi2s2);
  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_RESET_GPIO_Port, ADC_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADC_RESET_Pin */
  GPIO_InitStruct.Pin = ADC_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADC_RESET_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
