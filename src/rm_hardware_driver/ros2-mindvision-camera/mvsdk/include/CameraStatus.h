
#ifndef __CAMERA_STATUS_DEF__
#define __CAMERA_STATUS_DEF__

typedef int CameraSdkStatus;


/*���õĺ�*/
#define SDK_SUCCESS(_FUC_)              ((_FUC_) == CAMERA_STATUS_SUCCESS)

#define SDK_UNSUCCESS(_FUC_)            ((_FUC_) != CAMERA_STATUS_SUCCESS)

#define SDK_UNSUCCESS_RETURN(_FUC_,RET) if((RET = (_FUC_)) != CAMERA_STATUS_SUCCESS)\
                                        {\
                                            return RET;\
                                        }

#define SDK_UNSUCCESS_BREAK(_FUC_)      if((_FUC_) != CAMERA_STATUS_SUCCESS)\
                                        {\
                                            break;\
                                        }


/* ���ô���  */

#define CAMERA_STATUS_SUCCESS                    0   // �����ɹ�
#define CAMERA_STATUS_FAILED                    -1   // ����ʧ��
#define CAMERA_STATUS_INTERNAL_ERROR            -2   // �ڲ�����
#define CAMERA_STATUS_UNKNOW                    -3   // δ֪����
#define CAMERA_STATUS_NOT_SUPPORTED             -4   // ��֧�ָù���
#define CAMERA_STATUS_NOT_INITIALIZED           -5   // ��ʼ��δ���
#define CAMERA_STATUS_PARAMETER_INVALID         -6   // ������Ч
#define CAMERA_STATUS_PARAMETER_OUT_OF_BOUND    -7   // ����Խ��
#define CAMERA_STATUS_UNENABLED                 -8   // δʹ��
#define CAMERA_STATUS_USER_CANCEL               -9   // �û��ֶ�ȡ���ˣ�����roi�����ȡ��������
#define CAMERA_STATUS_PATH_NOT_FOUND            -10  // ע�����û���ҵ���Ӧ��·��
#define CAMERA_STATUS_SIZE_DISMATCH             -11  // ���ͼ�����ݳ��ȺͶ���ĳߴ粻ƥ��
#define CAMERA_STATUS_TIME_OUT                  -12  // ��ʱ����
#define CAMERA_STATUS_IO_ERROR                  -13  // Ӳ��IO����
#define CAMERA_STATUS_COMM_ERROR                -14  // ͨѶ����
#define CAMERA_STATUS_BUS_ERROR                 -15  // ���ߴ���
#define CAMERA_STATUS_NO_DEVICE_FOUND           -16  // û�з����豸
#define CAMERA_STATUS_NO_LOGIC_DEVICE_FOUND     -17  // δ�ҵ��߼��豸
#define CAMERA_STATUS_DEVICE_IS_OPENED          -18  // �豸�Ѿ���
#define CAMERA_STATUS_DEVICE_IS_CLOSED          -19  // �豸�Ѿ��ر�
#define CAMERA_STATUS_DEVICE_VEDIO_CLOSED       -20  // û�д��豸��Ƶ������¼����صĺ���ʱ����������Ƶû�д򿪣���ط��ظô���
#define CAMERA_STATUS_NO_MEMORY                 -21  // û���㹻ϵͳ�ڴ�
#define CAMERA_STATUS_FILE_CREATE_FAILED        -22  // �����ļ�ʧ��
#define CAMERA_STATUS_FILE_INVALID              -23  // �ļ���ʽ��Ч
#define CAMERA_STATUS_WRITE_PROTECTED           -24  // д����������д
#define CAMERA_STATUS_GRAB_FAILED               -25  // ���ݲɼ�ʧ��
#define CAMERA_STATUS_LOST_DATA                 -26  // ���ݶ�ʧ��������
#define CAMERA_STATUS_EOF_ERROR                 -27  // δ���յ�֡������
#define CAMERA_STATUS_BUSY                      -28  // ��æ(��һ�β������ڽ�����)���˴β������ܽ���
#define CAMERA_STATUS_WAIT                      -29  // ��Ҫ�ȴ�(���в���������������)�������ٴγ���trf
#define CAMERA_STATUS_IN_PROCESS                -30  // ���ڽ��У��Ѿ���������
#define CAMERA_STATUS_IIC_ERROR                 -31  // IIC�������
#define CAMERA_STATUS_SPI_ERROR                 -32  // SPI�������
#define CAMERA_STATUS_USB_CONTROL_ERROR         -33  // USB���ƴ������
#define CAMERA_STATUS_USB_BULK_ERROR            -34  // USB BULK�������
#define CAMERA_STATUS_SOCKET_INIT_ERROR         -35  // ���紫���׼���ʼ��ʧ��
#define CAMERA_STATUS_GIGE_FILTER_INIT_ERROR    -36  // ��������ں˹���������ʼ��ʧ�ܣ������Ƿ���ȷ��װ���������������°�װ��
#define CAMERA_STATUS_NET_SEND_ERROR            -37  // �������ݷ��ʹ���
#define CAMERA_STATUS_DEVICE_LOST               -38  // ���������ʧȥ���ӣ�������ⳬʱ
#define CAMERA_STATUS_DATA_RECV_LESS            -39  // ���յ����ֽ������������
#define CAMERA_STATUS_FUNCTION_LOAD_FAILED      -40  // ���ļ��м��س���ʧ��
#define CAMERA_STATUS_CRITICAL_FILE_LOST        -41  // ����������������ļ���ʧ��
#define CAMERA_STATUS_SENSOR_ID_DISMATCH        -42  // �̼��ͳ���ƥ�䣬ԭ���������˴���Ĺ̼���
#define CAMERA_STATUS_OUT_OF_RANGE              -43  // ����������Ч��Χ��
#define CAMERA_STATUS_REGISTRY_ERROR            -44  // ��װ����ע����������°�װ���򣬻������а�װĿ¼Setup/Installer.exe
#define CAMERA_STATUS_ACCESS_DENY               -45  // ��ֹ���ʡ�ָ������Ѿ�����������ռ��ʱ����������ʸ�������᷵�ظ�״̬��(һ��������ܱ��������ͬʱ����)
#define CAMERA_STATUS_CAMERA_NEED_RESET         -46  // ��ʾ�����Ҫ��λ���������ʹ�ã���ʱ��������ϵ�������������������ϵͳ�󣬱������ʹ�á�
#define CAMERA_STATUS_ISP_MOUDLE_NOT_INITIALIZED -47 // ISPģ��δ��ʼ��
#define CAMERA_STATUS_ISP_DATA_CRC_ERROR        -48  // ����У�����
#define CAMERA_STATUS_MV_TEST_FAILED            -49  // ���ݲ���ʧ��
#define CAMERA_STATUS_INTERNAL_ERR1             -50	 // �ڲ�����1
#define CAMERA_STATUS_U3V_NO_CONTROL_EP			-51	 // U3V���ƶ˵�δ�ҵ�
#define CAMERA_STATUS_U3V_CONTROL_ERROR			-52	 // U3V����ͨѶ����
#define CAMERA_STATUS_INVALID_FRIENDLY_NAME		-53	 ///< \~chinese ��Ч���豸���������ﲻ�ܰ��������ַ�(\/:*?"<>|") \~english Invalid device name, the name cannot contain the following characters (\/:*?"<>|")
#define CAMERA_STATUS_FORMAT_ERROR				-54	 ///< \~chinese ��ʽ���� \~english Format error
#define CAMERA_STATUS_PCIE_OPEN_ERROR			-55  ///< \~chinese PCIE�豸��ʧ�� \~english PCIE device open failed
#define CAMERA_STATUS_PCIE_COMM_ERROR			-56  ///< \~chinese PCIE�豸ͨѶʧ�� \~english PCIE device communication failed
#define CAMERA_STATUS_PCIE_DDR_ERROR			-57  ///< \~chinese PCIE DDR���� \~english PCIE DDR error



//��AIA�ƶ��ı�׼��ͬ
/*#define CAMERA_AIA_SUCCESS                        0x0000 */
#define CAMERA_AIA_PACKET_RESEND                          0x0100 //��֡��Ҫ�ش�
#define CAMERA_AIA_NOT_IMPLEMENTED                        0x8001 //�豸��֧�ֵ�����
#define CAMERA_AIA_INVALID_PARAMETER                      0x8002 //��������Ƿ�
#define CAMERA_AIA_INVALID_ADDRESS                        0x8003 //���ɷ��ʵĵ�ַ
#define CAMERA_AIA_WRITE_PROTECT                          0x8004 //���ʵĶ��󲻿�д
#define CAMERA_AIA_BAD_ALIGNMENT                          0x8005 //���ʵĵ�ַû�а���Ҫ�����
#define CAMERA_AIA_ACCESS_DENIED                          0x8006 //û�з���Ȩ��
#define CAMERA_AIA_BUSY                                   0x8007 //�������ڴ�����
#define CAMERA_AIA_DEPRECATED                             0x8008 //0x8008-0x0800B  0x800F  ��ָ���Ѿ�����
#define CAMERA_AIA_PACKET_UNAVAILABLE                     0x800C //����Ч
#define CAMERA_AIA_DATA_OVERRUN                           0x800D //���������ͨ�����յ������ݱ���Ҫ�Ķ�
#define CAMERA_AIA_INVALID_HEADER                         0x800E //���ݰ�ͷ����ĳЩ������Э�鲻ƥ��
#define CAMERA_AIA_PACKET_NOT_YET_AVAILABLE               0x8010 //ͼ��ְ����ݻ�δ׼���ã������ڴ���ģʽ��Ӧ�ó�����ʳ�ʱ
#define CAMERA_AIA_PACKET_AND_PREV_REMOVED_FROM_MEMORY    0x8011 //��Ҫ���ʵķְ��Ѿ������ڡ��������ش�ʱ�����Ѿ����ڻ�������
#define CAMERA_AIA_PACKET_REMOVED_FROM_MEMORY             0x8012 //CAMERA_AIA_PACKET_AND_PREV_REMOVED_FROM_MEMORY
#define CAMERA_AIA_NO_REF_TIME                            0x0813 //û�вο�ʱ��Դ��������ʱ��ͬ��������ִ��ʱ
#define CAMERA_AIA_PACKET_TEMPORARILY_UNAVAILABLE         0x0814 //�����ŵ��������⣬��ǰ�ְ���ʱ�����ã����Ժ���з���
#define CAMERA_AIA_OVERFLOW                               0x0815 //�豸�����������ͨ���Ƕ�������
#define CAMERA_AIA_ACTION_LATE                            0x0816 //����ִ���Ѿ�������Ч��ָ��ʱ��
#define CAMERA_AIA_ERROR                                  0x8FFF //����





#endif
