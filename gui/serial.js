/* MoRo 2025-2 @ Hongik Univ.
Web Serial wrapper (Chrome / Edge). Used by sketch.js: connect(), disconnect(), send(str), isConnected()
*/

class SerialLink {
  constructor(baud = 115200){
    this.baud = baud;
    this.port = null;
    this.writer = null;
    this.encoder = new TextEncoder();
  }
  isConnected(){ return this.writer !== null; }
  async connect(){
    if(!('serial' in navigator)) throw new Error('Web Serial not supported (use Chrome or Edge)');
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: this.baud });
    this.writer = this.port.writable.getWriter();
  }
  async disconnect(){
    const { port, writer } = this;
    this.writer = null; this.port = null;
    try{ if(writer){ await writer.close(); } }catch(e){}
    try{ if(port){ await port.close(); } }catch(e){}
  }
  send(str){
    if(!this.writer) return;
    this.writer.write(this.encoder.encode(str)).catch(() => {});
  }
}
