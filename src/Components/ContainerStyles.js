import styled from 'styled-components';

export const Container = styled.div`
  display: flex;
  flex-direction: column;
  height: 70vh;
  /* border: 2px maroon solid; */
  min-width: 150px;
  max-width: 150px;
  user-select: none;
  background: rgba(255,255,255,0.1);
`
export const Rows = styled.div`
  /* height: 500px; */
  margin-top: 3px;
  padding-right: 3px;
  padding-left: 3px;
  overflow-y: scroll;
  &::-webkit-scrollbar {
    width: 10px;
  };
  &::-webkit-scrollbar-thumb {
    background-color: #9b6a2f;
  }
  &::-webkit-scrollbar-track {
    background-color: black;
  }
`
export const HistoryCount = styled.div`
  font-size: 12px;
  margin-top: auto;
  padding: 5px;
  color: yellow;
  font-weight: 200;
`
export const Title = styled.div`
  text-align: center;
  color: grey;
  font-weight: 200;
`