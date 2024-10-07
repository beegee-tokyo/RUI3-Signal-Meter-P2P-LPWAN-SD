/**
 * @file sd-card.cpp
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief SD card functions
 * @version 0.1
 * @date 2024-09-26
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "app.h"
#include <SD.h> //http://librarymanager/All#SD

/** Forward declarations */
void dir_sd(File dir);
void dump_sd_file(const char *path);

/** Pointer to current log file */
File log_file;

/**
 * @brief Initialize SD card
 * 
 * @return true SD card found
 * @return false no SD card found
 */
bool init_sd(void)
{
	digitalWrite(WB_IO2, HIGH);
	delay(50);

	// #if MY_DEBUG > 0

	// 	Sd2Card card;
	// 	SdVolume volume;

	// 	if (!card.init(SPI_QUARTER_SPEED, WB_SPI_CS)) // SPI_QUARTER_SPEED SPI_HALF_SPEED SPI_FULL_SPEED
	// 	{
	// 		MYLOG("SD", "Card Init Failed! Please make sure the card is inserted!");
	// 		return false;
	// 	}

	// 	switch (card.type())
	// 	{
	// 	case SD_CARD_TYPE_SD1:
	// 		MYLOG("SD", "Cardtype SD1");
	// 		break;
	// 	case SD_CARD_TYPE_SD2:
	// 		MYLOG("SD", "Cardtype SD2");
	// 		break;
	// 	case SD_CARD_TYPE_SDHC:
	// 		MYLOG("SD", "Cardtype SDHC");
	// 		break;
	// 	default:
	// 		MYLOG("SD", "Cardtype inknown");
	// 	}

	// 	if (!volume.init(card))
	// 	{
	// 		MYLOG("SD", "Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
	// 		return false;
	// 	}

	// 	MYLOG("SD", "Clusters: %ld", volume.clusterCount());
	// 	MYLOG("SD", "Blocks x Cluster: %d", volume.blocksPerCluster());

	// 	MYLOG("SD", "Total Blocks: %ld", volume.blocksPerCluster() * volume.clusterCount());

	// 	// print the type and size of the first FAT-type volume
	// 	uint32_t volumesize;
	// 	MYLOG("SD", "Volume type is:    FAT%d", volume.fatType(), DEC);

	// 	volumesize = volume.blocksPerCluster(); // clusters are collections of blocks
	// 	volumesize *= volume.clusterCount();	// we'll have a lot of clusters
	// 	volumesize /= 2;						// SD card blocks are always 512 bytes (2 blocks are 1 KB)
	// 	MYLOG("SD", "Volume size (KB): %ld", volumesize);
	// 	MYLOG("SD", "Volume size (MB): %ld", volumesize / 1024);
	// 	MYLOG("SD", "Volume size (GB): %.2f", (float)volumesize / 1024 / 1024.0);
	// #endif

	if (!SD.begin(WB_SPI_CS))
	{
		MYLOG("SD", "SD begin failed.\nMake sure you've formatted the card and it is inserted");
		return false;
	}

	// File dir_file = SD.open("/", FILE_WRITE);
	// dir_sd(dir_file);
	// dir_file.close();

	// uint16_t file_num = 0;
	// sprintf((char *)file_name, "%04d-log.csv", file_num);
	// while (true)
	// {
	// 	if (SD.exists((const char *)file_name))
	// 	{
	// 		MYLOG("SD", "Content of %s:", file_name);
	// 		dump_sd_file((const char *)file_name);
	// 		file_num++;
	// 		sprintf((char *)file_name, "%04d-log.csv", file_num);
	// 	}
	// 	else
	// 	{
	// 		break;
	// 	}
	// }

	SD.end();
	return true;
}

/**
 * @brief Send directory of a folder to the Serial port
 * 
 * @param dir 
 */
void dir_sd(File dir)
{
	digitalWrite(WB_IO2, HIGH);
	delay(50);
	while (true)
	{

		log_file = dir.openNextFile();
		if (!log_file)
		{
			// no more files
			break;
		}
		Serial.print(log_file.name());
		if (log_file.isDirectory())
		{
			Serial.println("/");
			dir_sd(log_file);
		}
		else
		{
			// files have sizes, directories do not
			Serial.print("\t\t");
			Serial.println(log_file.size(), DEC);
		}
		log_file.close();
	}
}

/**
 * @brief Send content of all files to the Serial port
 * 
 */
void dump_all_sd_files(void)
{
	SD.begin(WB_SPI_CS);
	uint16_t file_num = 0;
	sprintf((char *)file_name, "%04d-log.csv", file_num);
	while (true)
	{
		if (SD.exists((const char *)file_name))
		{
			// MYLOG("SD", "Content of %s:", file_name);
			Serial.println("=====================================================");
			Serial.printf("%s\r\n", file_name);
			log_file = SD.open((const char *)file_name, FILE_READ); // re-open the file for reading.
			if (log_file)
			{
				// MYLOG("SD", "=====================================================");

				while (log_file.available())
				{
					Serial.write(log_file.read()); // read from the file until there's nothing else in it.
					delay(5);
				}
				log_file.close(); // close the file.
				// MYLOG("SD", "=====================================================");
				Serial.println("=====================================================");
				Serial.flush();
			}
			else
			{
				MYLOG("SD", "Failed to open file for reading."); // if the file didn't open, print an error.
			}

			file_num++;
			sprintf((char *)file_name, "%04d-log.csv", file_num);
			MYLOG("SD", "Next file %s", file_name);
		}
		else
		{
			break;
		}
	}

	SD.end();
}

/**
 * @brief Send the content of a file to the Serial port
 * 
 * @param path Path as string
 */
void dump_sd_file(const char *path)
{
	digitalWrite(WB_IO2, HIGH);
	delay(50);
	MYLOG("SD", "Reading file: %s", path);

	SD.begin(WB_SPI_CS);

	log_file = SD.open(path, FILE_READ); // re-open the file for reading.
	if (log_file)
	{
		// MYLOG("SD", "=====================================================");

		while (log_file.available())
		{
			Serial.write(log_file.read()); // read from the file until there's nothing else in it.
			delay(5);
		}
		log_file.close(); // close the file.
						  // MYLOG("SD", "=====================================================");
	}
	else
	{
		MYLOG("SD", "Failed to open file for reading."); // if the file didn't open, print an error.
	}
	SD.end();
}

/**
 * @brief Erase all files on the SD card
 * 
 */
void clear_sd_file(void)
{
	digitalWrite(WB_IO2, HIGH);
	delay(50);

	SD.begin(WB_SPI_CS);
	uint16_t file_num = 0;
	sprintf((char *)file_name, "%04d-log.csv", file_num);
	while (true)
	{
		if (SD.exists((const char *)file_name))
		{
			MYLOG("SD", "Delete %s:", file_name);
			SD.remove((const char *)file_name);
			file_num++;
			sprintf((char *)file_name, "%04d-log.csv", file_num);
			MYLOG("SD", "Next file %s", file_name);
		}
		else
		{
			break;
		}
	}

	SD.end();
}

/**
 * @brief Create a new file on the SD card.
 * 		Checks available files and generates a new file name
 * 
 * @return true File created
 * @return false File could not be created
 */
bool create_sd_file(void)
{
	digitalWrite(WB_IO2, HIGH);
	delay(50);

	SD.begin(WB_SPI_CS);

	File dir = SD.open("/", FILE_READ);
	uint16_t file_num = 0;
	sprintf((char *)file_name, "%04d-log.csv", file_num);
	if (dir)
	{
		while (true)
		{

			log_file = dir.openNextFile();
			if (!log_file)
			{
				// no more files
				MYLOG("SD", "New filename = %s", file_name);
				break;
			}
			if (!log_file.isDirectory())
			{
				char *last_file_name = log_file.name();
				last_file_name[4] = 0x00;
				file_num = atol(last_file_name);
				sprintf((char *)file_name, "%04d-log.csv", file_num + 1);
				MYLOG("SD", "Found %s", log_file.name());
			}
			log_file.close();
		}
		dir.close();
	}
	else
	{
		MYLOG("SD", "Can't open root");
	}

	log_file = SD.open((const char *)file_name, FILE_WRITE);
	if (log_file)
	{
		MYLOG("SD", "Writing Header to %s", file_name);
		if (g_custom_parameters.test_mode == MODE_LINKCHECK)
		{
			if (g_custom_parameters.location_on)
			{
				log_file.println("\"time\";\"Mode\";\"Gw\";\"Lat\";\"Lng\";\"RX RSSI\";\"RX SNR\";\"Demod\";\"Lost\"");
			}
			else
			{
				log_file.println("\"time\";\"Mode\";\"Gw\";\"RX RSSI\";\"RX SNR\";\"Demod\";\"Lost\"");
			}
		}
		else if (g_custom_parameters.test_mode == MODE_FIELDTESTER)
		{
			log_file.println("\"time\";\"Mode\";\"Gw\";\"Lat\";\"Lng\";\"min RSSI\";\"max RSSI\";\"RX RSSI\";\"RX SNR\";\"min Dist\";\"max Dist\"");
		}
		else // P2P mode
		{
			if (g_custom_parameters.location_on)
			{
				log_file.println("\"time\";\"Mode\";\"Lat\";\"Lng\";\"RX RSSI\";\"RX SNR\"");
			}
			else
			{
				log_file.println("\"time\";\"Mode\";\"RX RSSI\";\"RX SNR\"");
			}
		}
		log_file.flush();
		log_file.close();
		SD.end();

		return true;
	}
	SD.end();

	return false;
}

/**
 * @brief Write to the current log file
 * 
 */
void write_sd_entry(void)
{

	digitalWrite(WB_IO2, HIGH);
	delay(50);

	SD.begin(WB_SPI_CS);

	char line_entry[512];
	if (!SD.exists((const char *)file_name))
	{
		MYLOG("SD", "Can't find %s", file_name);
	}
	else
	{
		MYLOG("SD", "Found %s", file_name);
	}

	log_file = SD.open((const char *)file_name, FILE_WRITE);
	if (log_file)
	{

		if (g_custom_parameters.test_mode == MODE_LINKCHECK)
		{
			if (g_custom_parameters.location_on)
			{
				// log_file.println("\"time\";\"Mode\";\"Gw\";\"Lat\";\"Lng\";\"RX RSSI\";\"RX SNR\";\"Demod\";\"Lost\"");
				snprintf(line_entry, 511, "%04d-%02d-%02d-%02d-%02d;%d;%d;%.6f;%.6f;%d;%d;%d;%d",
						 result.year, result.month, result.day, result.hour, result.min,
						 result.mode, result.gw,
						 result.lat, result.lng,
						 result.rx_rssi,
						 result.rx_snr,
						 result.demod, result.lost);
			}
			else
			{
				snprintf(line_entry, 511, "%04d-%02d-%02d-%02d-%02d;%d;%d;%d;%d;%d;%d",
						 result.year, result.month, result.day, result.hour, result.min,
						 result.mode, result.gw,
						 result.rx_rssi,
						 result.rx_snr,
						 result.demod, result.lost);
			}
		}
		else if (g_custom_parameters.test_mode == MODE_FIELDTESTER)
		{
			// log_file.println("\"time\";\"Mode\";\"Gw\";\"Lat\";\"Lng\";\"min RSSI\";\"max RSSI\";\"RX RSSI\";\"RX SNR\";\"min Dist\";\"max Dist\"");
			snprintf(line_entry, 511, "%04d-%02d-%02d-%02d-%02d;%d;%d;%.6f;%.6f;%d;%d;%d;%d;%d;%d",
					 result.year, result.month, result.day, result.hour, result.min,
					 result.mode, result.gw,
					 result.lat, result.lng,
					 result.min_rssi, result.max_rssi, result.rx_rssi,
					 result.rx_snr,
					 result.min_dst, result.max_dst);
		}
		else // LoRa P2P
		{
			if (g_custom_parameters.location_on)
			{
				// log_file.println("\"time\";\"Mode\";\"Lat\";\"Lng\";\"RX RSSI\";\"RX SNR\"");
				snprintf(line_entry, 511, "%04d-%02d-%02d-%02d-%02d;%d;%.6f;%.6f;%d;%d",
						 result.year, result.month, result.day, result.hour, result.min,
						 result.mode,
						 result.lat, result.lng,
						 result.rx_rssi,
						 result.rx_snr);
			}
			else
			{
				// log_file.println("\"time\";\"Mode\";\"Lat\";\"Lng\";\"RX RSSI\";\"RX SNR\"");
				snprintf(line_entry, 511, "%04d-%02d-%02d-%02d-%02d;%d;%d;%d",
						 result.year, result.month, result.day, result.hour, result.min,
						 result.mode,
						 result.rx_rssi,
						 result.rx_snr);
			}
		}
		// snprintf(line_entry, 511, "%04d-%02d-%02d-%02d-%02d;%d;%d;%.6f;%.6f;%d;%d;%d;%d;%d;%d;%d;%d",
		// 		 result.year, result.month, result.day, result.hour, result.min,
		// 		 result.mode, result.gw,
		// 		 result.lat, result.lng,
		// 		 result.min_rssi, result.max_rssi, result.rx_rssi,
		// 		 result.rx_snr,
		// 		 result.min_dst, result.max_dst,
		// 		 result.demod, result.lost);
		MYLOG("SD", "Writing:\r\n%s", line_entry);
		log_file.println(line_entry);
		log_file.flush();
		log_file.close();
	}
	else
	{
		MYLOG("SD", "Error writing to %s", file_name);
	}

	SD.end();
	return;
}