////////////////////////////////////////////////////////////////////////////////
//	���ܣ� CRCУ��ͷ�ļ�
//
//	�汾��V1.0
//  ������
//												�Ͼ�����ѧ΢����������
//												   2016.01.06
////////////////////////////////////////////////////////////////////////////////

#ifndef __CRC32_H
#define __CRC32_H

/**
 * Calculate single step of crc32
 */
uint32_t chksum_crc32_step(uint32_t crc, uint8_t byte);

/**
 * Caluclate crc32 of a block
 */
uint32_t chksum_crc32(uint8_t *block, unsigned int length);

#endif /* __CRC32_H */
