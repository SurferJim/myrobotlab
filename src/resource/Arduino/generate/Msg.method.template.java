
	public void %name%(%javaMethodParameters%) {
		try {
			write(MAGIC_NUMBER);
			write(%javaWriteMsgSize%); // size
%javaWrite% 
	  } catch (Exception e) {
	  			serial.error(e);
	  }
	}
