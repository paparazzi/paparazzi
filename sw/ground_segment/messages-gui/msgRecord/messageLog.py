# Copyright (C) 2024 Mael FEURGARD <mael.feurgard@enac.fr>
# 
# This file is part of messages_python.
# 
# messages_python is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# messages_python is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with messages_python.  If not, see <https://www.gnu.org/licenses/>.

import dataclasses
import typing
import time
from functools import total_ordering


from collections import deque

from pprzlink.message import PprzMessage,PprzMessageField

@dataclasses.dataclass
class MessageIndex:
    sender_id:typing.Optional[int] # Use None for Unknown sender
    class_id:int
    message_id:int
    
    def pprzMsg(self) -> PprzMessage:
        return PprzMessage(self.class_id,self.message_id)
    
@dataclasses.dataclass
class FieldIndex:
    msgIndex:MessageIndex
    field:str
    array_index:typing.Optional[int] = None
    
    @staticmethod
    def from_ints(s_id:int,c_id:int,m_id:int,f_str:str,a_id:typing.Optional[int]=None):
        return FieldIndex(MessageIndex(s_id,c_id,m_id),f_str,a_id)
    
    @property
    def sender_id(self) -> int:
        return self.msgIndex.sender_id
        
    @property
    def class_id(self) -> int:
        return self.msgIndex.class_id
    
    @property
    def message_id(self) -> int:
        return self.msgIndex.message_id
    
    def pprzMsg(self) -> PprzMessage:
        return self.msgIndex.pprzMsg()


@total_ordering
class TimedPprzMessage():
    def __init__(self, msg:PprzMessage, t:typing.Optional[int]=None):
        self.msg = msg
        self._timestamp = time.time_ns() if t is None else t
    
    @property
    def fieldnames(self) -> list[str]:
        return self.msg.fieldnames
    
    def get_full_field(self, fieldname:str) -> PprzMessageField:
        return self.msg.get_full_field(fieldname)
    
    def __getattr__(self,key:str):
        return self.msg.__getattr__(key)
    
    def __getitem__(self,key:str):
        return self.msg.__getitem__(key)
    
    @property
    def name(self) -> str:
        return self.msg.name
    
    @property
    def msg_class(self) -> str:
        return self.msg.msg_class
    
    @property
    def msg_id(self) -> int:
        return self.msg.msg_id
    
    @property
    def class_id(self) -> int:
        return self.msg.class_id
        
    @property
    def timestamp(self) -> int:
        """Get the message reception timestamp (in ns by default)."""
        return self._timestamp
    
    # @property.setter
    # def timestamp(self,t:int) -> int:
    #     """Set the message reception timestamp."""
    #     self._timestamp = t
        
    def timeit(self) -> int:
        """ Set the timestamp to NOW (in ns, since Epoch), and returns it"""
        self._timestamp = time.time_ns()
        return self._timestamp
    
    def __eq__(self,other)->bool:
        return self._timestamp == other._timestamp and self.name == other.name
    
    def __lt__(self,other)->bool:
        return self.timestamp < other.timestamp
    
    def index(self) -> MessageIndex:
        return MessageIndex(None,self.class_id,self.msg_id)
        

class NoMessageError(Exception):
    def __init__(self, *args: object) -> None:
        super().__init__("No messages in this log")

class GroupByError(Exception):
    pass

class MessageLog():
    def __init__(self,size:int=10):
        self.queue:typing.Deque[TimedPprzMessage] = deque(maxlen=size)
        self.period:typing.Optional[float] = None 
        
        self.__groupBy:typing.Optional[str] = None
        self.__groups:dict[typing.Any,MessageLog] = dict()
        
        
    def updateSize(self,s:int):
        self.queue = deque(self.queue,maxlen=s)
        for m in self.__groups.values():
            m.updateSize(s)
        
    ########## Subgroup management ##########
     
    def grouped(self) -> bool:
        return not(self.__groupBy is None)
    
    def groupedBy(self) -> typing.Optional[str]:
        return self.__groupBy
    
    def groupBy(self,s:typing.Optional[str]):
        if s != self.__groupBy:
            self.__groups.clear()
        
        if s is not None:
            field = self.get_full_field(s)
            if field.array_type:
                raise GroupByError("Cannot group by an array type")
            
        self.__groupBy = s
    
    def clearGroupBy(self):
        self.groupBy(None)
    
    def subgroup(self,val):
        try:
            return self.__groups[val]
        except KeyError:
            return None
        
    def subgroups(self):
        return self.__groups
                 
    ########## Manage messages ##########
                 
    def addMessage(self,msg:TimedPprzMessage):
        if len(self.queue) > 0:
            if self.period is None:
                self.period = msg.timestamp - self.queue[0].timestamp
            else:
                self.period = (msg.timestamp - self.queue[0].timestamp + self.period)/2
        
        self.queue.appendleft(msg)
        
        if self.__groupBy is not None:
            field = msg.get_full_field(self.__groupBy)
            try:
                sublog = self.__groups[field.val]
            except KeyError:
                sublog = MessageLog(self.queue.maxlen)
                self.__groups[field.val] = sublog
            sublog.addMessage(msg)
                
        
    def addMessages(self,msgs:typing.Iterable[TimedPprzMessage]):
        sorted_msgs = sorted(msgs)
        if self.period is None:
            self.period = (sorted_msgs[-1] - sorted_msgs[0])/len(sorted_msgs)
        else:
            self.period = ((sorted_msgs[-1] - sorted_msgs[0])/len(sorted_msgs) + self.period)/2
        self.queue.extendleft(sorted_msgs)
        
        if self.__groupBy is not None:
            for m in msgs:
                field = m.get_full_field(self.__groupBy)
                try:
                    sublog = self.__groups[field.val]
                except KeyError:
                    sublog = MessageLog(self.queue.maxlen)
                    self.__groups[field.val] = sublog
                    
                sublog.addMessage(m)
                

    ########## Accessors ##########                
                
    def newest(self) -> TimedPprzMessage:
        try:
            return self.queue[0]
        except IndexError:
            raise NoMessageError()
    
    def meanFreq(self) -> float:
        if self.period is None:
            raise NoMessageError()
        else:
            return 10e9/self.period #ns to s, then s to Hz
    
    def msg_name(self) -> str:
        return self.newest().name
    
    def msg_class(self) -> str:
        return self.newest().msg_class
    
    def msg_id(self) -> int:
        return self.newest().msg_id
    
    def class_id(self) -> int:
        return self.newest().class_id
    
    def sample_count(self) -> int:
        return len(self.queue)
    
    def get_full_field(self, fieldname:str) -> PprzMessageField:
        return self.newest().get_full_field(fieldname)
    
    def fieldnames(self) -> list[str]:
        return self.newest().fieldnames
    
    def index(self) -> MessageIndex:
        msg = self.newest()
        return msg.index()
    

    
    
    
            
    