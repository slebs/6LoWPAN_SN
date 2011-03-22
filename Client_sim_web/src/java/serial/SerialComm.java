/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package serial;

import at.htlklu.elektronik.schnittstellen.SerielleSchnittstelle;
import at.htlklu.elektronik.schnittstellen.StringEvent;
import at.htlklu.elektronik.schnittstellen.StringListener;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author simon
 */
public class SerialComm implements StringListener {

    private static final SerialComm instance = new SerialComm();
    // Commands to be send to RF-Modul (Coordinator)
    //----------------------------------------------------
    public static final String GETDATA = "#getdata";
    public static final String GETECHO = "#getecho";
    public static final String GETNODES = "#getnodes";
    public static final int COMMAND_GETDATA = 0;
    public static final int COMMAND_GETECHO = 1;
    public static final int COMMAND_GETNODES = 2;
    public static final int COMMAND_CLOSE = 3;
    //---------------------------------------------------
    // Commands reveivec from RF-Modul (Coordinator)
    //----------------------------------------------------
    private static final String BODATA = "#BOData";
    private static final String EODATA = "#EOData";
    //---------------------------------------------------
    public static final int STATE_IDLE = 0;
    public static final int STATE_LISTENING = 1;
    SerielleSchnittstelle com;
    String strR;
    int state;
    ArrayList<String> dataList = new ArrayList<String>();

    /*
     * sends the String with a delay between every charackter
     * This needs to be done to avoid timing problems on the micro controller
     */
    public void sendString_d(String s) {
        try {
            char[] ca = s.toCharArray();
            com.sendByte(' ');
            Thread.sleep(10);
            for (int j = 0; j < ca.length; j++) {
                com.sendByte(ca[j]);
                Thread.sleep(2);
            }
            com.sendByte('\n');
            System.out.println("string sent");
        } catch (InterruptedException ex) {
            Logger.getLogger(SerialComm.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    private SerialComm() {
        com = new SerielleSchnittstelle();
    }

    public void sendCommand(int command, int nodeAddr) {
        if (com.isConnected()) {
            switch (command) {
                case COMMAND_GETDATA:
                    sendString_d(GETDATA + " " + nodeAddr);
                    break;
                case COMMAND_GETECHO:
                    sendString_d(GETECHO + " " + nodeAddr);
                    break;
                case COMMAND_GETNODES:
                    sendString_d(GETNODES);
                    break;
                default:
                    System.out.println("unkown command");
            }
        } else {
        }
    }

    public void disconnect() {
        com.disconnect();
    }

    public void stringReceived(StringEvent se) {
        strR = se.getStringReceived();

        if (strR.equals(BODATA)) {
            System.out.println("got command " + BODATA);
            state = STATE_LISTENING;
        } else if (strR.equals(EODATA)) {
            System.out.println("got command " + EODATA);
        } else if (state == STATE_LISTENING) {
            System.out.println("reveived DATA from node: " + strR);
            addData(strR);
            state = STATE_IDLE;
        } else {
            System.out.println("from Coord: " + strR);
        }
    }

    public static SerialComm getInstance() {
        return instance;
    }

    private void addData(String strR) {
        dataList.add(strR);
    }

    public void connect(String portName, int baud) {
        if (com.isConnected()) {
            com.disconnect();
        }
        com = null;
        com = new SerielleSchnittstelle(portName);
        com.setBaudRate(baud);
        com.setStringDelimiter(SerielleSchnittstelle.DELIMITER_LF);
        com.addStringListener(this);
    }
    public String getPortName(){
        if(com.isConnected()){
            return com.getPortName();
        }
        return "/dev/ttyUSB0";
    }
    public int getBaud(){
        if(com.isConnected()){
            return com.getBaudRate();
        }
        return 115200;
    }
}
